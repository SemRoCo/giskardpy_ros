import abc
from abc import ABC
from typing import Tuple, List, Iterable, Optional

import numpy as np

import giskardpy.casadi_wrapper as cas
from giskardpy.configs.data_types import SupportedQPSolver
from giskardpy.exceptions import HardConstraintsViolatedException, InfeasibleException, QPSolverException
from giskardpy.utils import logging
from giskardpy.utils.utils import memoize


class QPSolver(ABC):
    solver_id: SupportedQPSolver
    qp_setup_function: cas.CompiledFunction
    # num_non_slack = num_non_slack
    retry_added_slack = 100
    retry_weight_factor = 100
    retries_with_relaxed_constraints = 5

    @abc.abstractmethod
    def __init__(self, weights: cas.Expression, g: cas.Expression, lb: cas.Expression, ub: cas.Expression,
                 A: cas.Expression, A_slack: cas.Expression, lbA: cas.Expression, ubA: cas.Expression,
                 E: cas.Expression, E_slack: cas.Expression, bE: cas.Expression):
        pass

    @profile
    def solve(self, substitutions: List[float], relax_hard_constraints: bool = False) -> np.ndarray:
        problem_data = self.qp_setup_function.fast_call(substitutions)
        split_problem_data = self.split_and_filter_results(problem_data, relax_hard_constraints)
        return self.solver_call(*split_problem_data)

    @profile
    def solve_and_retry(self, substitutions: List[float]) -> np.ndarray:
        """
        Calls solve and retries on exception.
        """
        try:
            return self.solve(substitutions)
        except QPSolverException as e:
            try:
                logging.loginfo(f'{e}; retrying with relaxed hard constraints')
                return self.solve(substitutions, relax_hard_constraints=True)
            except InfeasibleException as e2:
                if isinstance(e2, HardConstraintsViolatedException):
                    raise e2
                raise e

    @abc.abstractmethod
    def split_and_filter_results(self, problem_data: np.ndarray, relax_hard_constraints: bool = False) \
            -> List[np.ndarray]:
        pass

    @abc.abstractmethod
    def solver_call(self, *args, **kwargs) -> np.ndarray:
        pass

    @abc.abstractmethod
    def relaxed_problem_data_to_qpSWIFT_format(self, *args, **kwargs) -> List[np.ndarray]:
        pass

    @abc.abstractmethod
    def get_problem_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                                        np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        :return: weights, g, lb, ub, E, bE, A, lbA, ubA, weight_filter, bE_filter, bA_filter
        """


class QPSWIFTFormatter(QPSolver):

    @profile
    def __init__(self, weights: cas.Expression, g: cas.Expression, lb: cas.Expression, ub: cas.Expression,
                 E: cas.Expression, E_slack: cas.Expression, bE: cas.Expression,
                 A: cas.Expression, A_slack: cas.Expression, lbA: cas.Expression, ubA: cas.Expression):
        """
        min_x 0.5 x^T H x + g^T x
        s.t.  Ex = b
              Ax <= lb/ub
        combined matrix format:
                   |x|          1
             |---------------|-----|
        1    |    weights    |  0  |
        |b|  |      E        |  b  |
        |x|  |     -I        | -lb |
        |lbA||     -A        |-lbA |
        |x|  |      I        |  ub |
        |lbA||      A        | ubA |
             |---------------|-----|
        """
        self.num_eq_constraints = bE.shape[0]
        self.num_neq_constraints = lbA.shape[0]
        self.num_free_variable_constraints = lb.shape[0]
        self.num_eq_slack_variables = E_slack.shape[1]
        self.num_neq_slack_variables = A_slack.shape[1]
        self.num_slack_variables = self.num_eq_slack_variables + self.num_neq_slack_variables
        self.num_non_slack_variables = self.num_free_variable_constraints - self.num_slack_variables

        combined_problem_data = cas.zeros(1 + bE.shape[0] + lbA.shape[0] * 2 + self.num_free_variable_constraints * 2,
                                          self.num_free_variable_constraints + 1)
        self.weights_slice = (0, slice(None, -1))
        # self.g_slice = (1, slice(None, -1))
        offset = 1
        self.E_slice = (slice(offset, offset + self.num_eq_constraints), slice(None, E.shape[1]))
        self.E_slack_slice = (self.E_slice[0], slice(self.num_non_slack_variables, E.shape[1] + E_slack.shape[1]))
        self.E_E_slack_slice = (self.E_slice[0], slice(None, -1))
        self.bE_slice = (self.E_slice[0], -1)
        offset += self.num_eq_constraints

        self.nI_slice = (slice(offset, offset + self.num_free_variable_constraints),
                         slice(None, self.num_free_variable_constraints))
        self.nlb_slice = (self.nI_slice[0], -1)
        offset += self.num_free_variable_constraints

        self.nA_slice = (slice(offset, offset + self.num_neq_constraints), slice(None, A.shape[1]))
        self.nA_slack_slice = (self.nA_slice[0], slice(self.num_non_slack_variables + E_slack.shape[1], -1))
        self.nlbA_slice = (self.nA_slice[0], -1)
        offset += self.num_neq_constraints

        self.I_slice = (slice(offset, offset + self.num_free_variable_constraints),
                        slice(None, self.num_free_variable_constraints))
        self.ub_slice = (self.I_slice[0], -1)
        offset += self.num_free_variable_constraints

        self.A_slice = (slice(offset, None), self.nA_slice[1])
        self.A_slack_slice = (self.A_slice[0], self.nA_slack_slice[1])
        self.ubA_slice = (self.A_slice[0], -1)

        self.nI_I_nA_nA_slack_A_A_slack_slice = (slice(1 + self.num_eq_constraints, None), slice(None, -1))
        self.nlb_nlbA_ub_ubA_slice = (self.nI_I_nA_nA_slack_A_A_slack_slice[0], -1)

        combined_problem_data[self.weights_slice] = weights
        if self.num_eq_constraints > 0:
            combined_problem_data[self.E_slice] = E
            combined_problem_data[self.E_slack_slice] = E_slack
            combined_problem_data[self.bE_slice] = bE
        combined_problem_data[self.nI_slice] = -cas.eye(self.num_free_variable_constraints)
        combined_problem_data[self.nlb_slice] = -lb
        if self.num_neq_constraints > 0:
            combined_problem_data[self.nA_slice] = -A
            combined_problem_data[self.nA_slack_slice] = -A_slack
            combined_problem_data[self.nlbA_slice] = -lbA
        combined_problem_data[self.I_slice] = cas.eye(self.num_free_variable_constraints)
        combined_problem_data[self.ub_slice] = ub
        if self.num_neq_constraints > 0:
            combined_problem_data[self.A_slice] = A
            combined_problem_data[self.A_slack_slice] = A_slack
            combined_problem_data[self.ubA_slice] = ubA

        self.qp_setup_function = combined_problem_data.compile()

    @profile
    def split_and_filter_results(self, combined_problem_data: np.ndarray, relax_hard_constraints: bool = False) \
            -> Iterable[np.ndarray]:
        self.combined_problem_data = combined_problem_data
        self.weights = combined_problem_data[self.weights_slice]
        self.g = np.zeros(self.weights.shape)
        self.E = combined_problem_data[self.E_E_slack_slice]
        self.bE = combined_problem_data[self.bE_slice]
        self.nI_nA_I_A = combined_problem_data[self.nI_I_nA_nA_slack_A_A_slack_slice]
        self.nlb_nlbA_ub_ubA = combined_problem_data[self.nlb_nlbA_ub_ubA_slice]

        self.update_filters()
        self.apply_filters()
        if relax_hard_constraints:
            return self.relaxed_problem_data_to_qpSWIFT_format(self.weights, self.nI_nA_I_A, self.nlb_nlbA_ub_ubA)
        return self.problem_data_to_qpSWIFT_format(self.weights, self.nI_nA_I_A, self.nlb_nlbA_ub_ubA)

    @profile
    def problem_data_to_qpSWIFT_format(self, weights: np.ndarray, nI_nA_I_A: np.ndarray, nlb_nlbA_ub_ubA: np.ndarray) \
            -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        H = np.diag(weights)
        return H, self.g, self.E, self.bE, nI_nA_I_A, nlb_nlbA_ub_ubA

    @profile
    def relaxed_problem_data_to_qpSWIFT_format(self, weights: np.ndarray, nA_A: np.ndarray, nlb_ub_nlbA_ubA: np.ndarray) \
            -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        self.retries_with_relaxed_constraints -= 1
        if self.retries_with_relaxed_constraints <= 0:
            raise HardConstraintsViolatedException('Out of retries with relaxed hard constraints.')
        nlb_relaxed = nlb.copy()
        ub_relaxed = ub.copy()
        nlb_relaxed[self.num_non_slack_variables:] = self.retry_added_slack
        ub_relaxed[self.num_non_slack_variables:] = self.retry_added_slack
        try:
            relaxed_problem_data = self.problem_data_to_qpSWIFT_format(weights=weights,
                                                                       nA_A=nA_A,
                                                                       nlb=nlb_relaxed,
                                                                       ub=ub_relaxed,
                                                                       nlbA_ubA=nlb_ub_nlbA_ubA)
            xdot_full = self.solver_call(*relaxed_problem_data)
        except QPSolverException as e:
            self.retries_with_relaxed_constraints += 1
            raise e
        upper_violations = ub < xdot_full
        lower_violations = nlb < xdot_full
        if np.any(upper_violations) or np.any(lower_violations):
            weights[upper_violations | lower_violations] *= self.retry_weight_factor
            return self.problem_data_to_qpSWIFT_format(weights=weights,
                                                       nA_A=nA_A,
                                                       nlb=nlb_relaxed,
                                                       ub=ub_relaxed,
                                                       nlbA_ubA=nlbA_ubA)
        self.retries_with_relaxed_constraints += 1
        raise InfeasibleException('')

    @profile
    def update_filters(self):
        self.weight_filter = self.weights != 0
        self.weight_filter[:-self.num_slack_variables] = True
        slack_part = self.weight_filter[-(self.num_eq_slack_variables + self.num_neq_slack_variables):]
        bE_part = slack_part[:self.num_eq_slack_variables]
        bA_part = slack_part[self.num_eq_slack_variables:]

        self.bE_filter = np.ones(self.E.shape[0], dtype=bool)
        if len(bE_part) > 0:
            self.bE_filter[-len(bE_part):] = bE_part

        self.bA_filter_half = np.ones(int(self.nI_nA_I_A.shape[0] / 2), dtype=bool)
        if len(bA_part) > 0:
            self.bA_filter_half[-len(bA_part):] = bA_part
        self.bA_filter_half[:len(self.weight_filter)] = self.weight_filter
        self.bA_filter = np.concatenate((self.bA_filter_half, self.bA_filter_half))

    @profile
    def apply_filters(self):
        self.weights = self.weights[self.weight_filter]
        self.g = np.zeros(*self.weights.shape)
        self.E = self.E[self.bE_filter, :][:, self.weight_filter]
        self.bE = self.bE[self.bE_filter]
        self.nI_nA_I_A = self.nI_nA_I_A[:, self.weight_filter][self.bA_filter, :]
        self.nlb_nlbA_ub_ubA = self.nlb_nlbA_ub_ubA[self.bA_filter]

    @profile
    def get_problem_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray,
                                        np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        :return: weights, g, lb, ub, E, bE, A, lbA, ubA, weight_filter, bE_filter, bA_filter
        """
        weights = self.weights
        g = self.g
        I_length = len(weights)
        A_half = int(self.nI_nA_I_A.shape[0] / 2)
        I_A = self.nI_nA_I_A[A_half:, :]
        A = I_A[I_length:]

        nlb_nlbA = self.nlb_nlbA_ub_ubA[:A_half]
        ub_ubA = self.nlb_nlbA_ub_ubA[A_half:]
        lb = -nlb_nlbA[:I_length]
        lbA = -nlb_nlbA[I_length:]
        ub = ub_ubA[:I_length]
        ubA = ub_ubA[I_length:]

        E = self.E
        bE = self.bE
        return weights, g, lb, ub, E, bE, A, lbA, ubA, self.weight_filter, self.bE_filter, self.bA_filter_half[len(self.weight_filter):]

    @abc.abstractmethod
    def solver_call(self, H: np.ndarray, g: np.ndarray, E: np.ndarray, b: np.ndarray, A: np.ndarray, h: np.ndarray) \
            -> np.ndarray:
        pass
