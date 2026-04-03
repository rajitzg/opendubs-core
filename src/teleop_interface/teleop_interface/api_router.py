"""FastAPI router for controlling and querying rosbag logging state."""

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel
from interfaces.srv import LoggerCommand

# --- Pydantic Models ---
class LoggerResponse(BaseModel):
    """HTTP response model for logger command endpoints."""

    success: bool
    message: str

class StatusResponse(BaseModel):
    """HTTP response model for logger status endpoint."""

    is_recording: bool

# Create a FastAPI router
router = APIRouter()

# --- Endpoints ---

@router.post("/logger/start", response_model=LoggerResponse)
def start_logger(request: Request):
    """Start rosbag recording through the ROS logger service.

    Args:
        request (Request): FastAPI request containing app state with ROS node.

    Returns:
        LoggerResponse: Success message when recording starts.
    """
    # Access the ros_node injected into app state
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail="ROS node not initialized")
    
    success = ros_node.send_logger_command(LoggerCommand.Request.START_RECORDING)
    if success:
        return LoggerResponse(success=True, message="Logger started.")
    else:
        raise HTTPException(status_code=503, detail="Failed to start logger. Is the service running?")

@router.post("/logger/stop_and_save", response_model=LoggerResponse)
def stop_logger(request: Request):
    """Stop recording and persist bag files.

    Args:
        request (Request): FastAPI request containing app state with ROS node.

    Returns:
        LoggerResponse: Success message when recording is saved.
    """
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail="ROS node not initialized")
    
    success = ros_node.send_logger_command(LoggerCommand.Request.STOP_AND_SAVE_RECORDING)
    if success:
        return LoggerResponse(success=True, message="Logger stopped and saved.")
    else:
        raise HTTPException(status_code=503, detail="Failed to stop logger. Is the service running?")

@router.post("/logger/stop_and_discard", response_model=LoggerResponse)
def stop_discard_logger(request: Request):
    """Stop recording and discard temporary bag files.

    Args:
        request (Request): FastAPI request containing app state with ROS node.

    Returns:
        LoggerResponse: Success message when recording is discarded.
    """
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail="ROS node not initialized")
    
    success = ros_node.send_logger_command(LoggerCommand.Request.STOP_AND_DISCARD_RECORDING)
    if success:
        return LoggerResponse(success=True, message="Logger stopped and discarded.")
    else:
        raise HTTPException(status_code=503, detail="Failed to stop logger. Is the service running?")

@router.get("/logger/status", response_model=StatusResponse)
def get_logger_status(request: Request):
    """Return whether the recorder is currently active.

    Args:
        request (Request): FastAPI request containing app state with ROS node.

    Returns:
        StatusResponse: Current `is_recording` state.
    """
    ros_node = request.app.state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=500, detail="ROS node not initialized")
    return StatusResponse(is_recording=ros_node.is_recording)
