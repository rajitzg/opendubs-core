from fastapi import APIRouter
from fastapi import HTTPException
from fastapi import Request
from interfaces.srv import LoggerCommand
from pydantic import BaseModel

# --- Pydantic Models ---


class LoggerResponse(BaseModel):
    success: bool
    message: str


class StatusResponse(BaseModel):
    is_recording: bool


# Create a FastAPI router
router = APIRouter()

# --- Endpoints ---


@router.post('/logger/start', response_model=LoggerResponse)
def start_logger(request: Request):
    # Access the ros_node injected into app state
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail='ROS node not initialized')

    success = ros_node.send_logger_command(LoggerCommand.Request.START_RECORDING)
    if success:
        return LoggerResponse(success=True, message='Logger started.')
    else:
        raise HTTPException(
            status_code=503, detail='Failed to start logger. Is the service running?')


@router.post('/logger/stop_and_save', response_model=LoggerResponse)
def stop_logger(request: Request):
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail='ROS node not initialized')

    success = ros_node.send_logger_command(LoggerCommand.Request.STOP_AND_SAVE_RECORDING)
    if success:
        return LoggerResponse(success=True, message='Logger stopped and saved.')
    else:
        raise HTTPException(
            status_code=503, detail='Failed to stop logger. Is the service running?')


@router.post('/logger/stop_and_discard', response_model=LoggerResponse)
def stop_discard_logger(request: Request):
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail='ROS node not initialized')

    success = ros_node.send_logger_command(LoggerCommand.Request.STOP_AND_DISCARD_RECORDING)
    if success:
        return LoggerResponse(success=True, message='Logger stopped and discarded.')
    else:
        raise HTTPException(
            status_code=503, detail='Failed to stop logger. Is the service running?')


@router.get('/logger/status', response_model=StatusResponse)
def get_logger_status(request: Request):
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail='ROS node not initialized')
    return StatusResponse(is_recording=ros_node.is_recording)
