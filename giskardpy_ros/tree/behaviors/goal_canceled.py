from py_trees.common import Status

from giskardpy.data_types.exceptions import PreemptedException
from giskardpy_ros.tree.behaviors.action_server import ActionServerHandler
from giskardpy_ros.tree.behaviors.plugin import GiskardBehavior
from giskardpy.middleware import get_middleware
from giskardpy.utils.decorators import record_time
from giskardpy_ros.tree.blackboard_utils import raise_to_blackboard, GiskardBlackboard


class GoalCanceled(GiskardBehavior):
    @profile
    def __init__(self, action_server: ActionServerHandler):
        name = f'is \'{action_server.name}\' cancelled?'
        self.action_server = action_server
        super().__init__(name)

    @record_time
    @profile
    def update(self) -> Status:
        if (self.action_server.is_cancel_requested() and self.get_blackboard_exception() is None or
                not self.action_server.is_client_alive()):
            GiskardBlackboard().move_action_server.set_canceled()
            msg = f'\'{self.action_server.name}\' preempted'
            get_middleware().logerr(msg)
            raise_to_blackboard(PreemptedException(msg))
        if self.get_blackboard_exception() is not None:
            return Status.SUCCESS
        else:
            return Status.FAILURE
