""" Ballbot Main control loop """

from controllers import BodyController

class BallbotController(object):
  def __init__(self):
    self._body_controller = BodyController()

  def update_control(self):

    """
    " READ ROBOT DATA
    """
    