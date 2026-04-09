from dataclasses import dataclass, field
from enum import Enum
import uuid


class TaskStatus(Enum):
    PENDING = 'pending'
    IN_PROGRESS = 'in_progress'
    NAVIGATING_TO_PICKUP = 'navigating_to_pickup'
    PICKING_UP = 'picking_up'
    NAVIGATING_TO_DELIVERY = 'navigating_to_delivery'
    DROPPING_OFF = 'dropping_off'
    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    FAILED = 'failed'


@dataclass
class DeliveryOrder:
    pickup_station: str
    delivery_station: str
    priority: int = 1
    order_id: str = field(default_factory=lambda: str(uuid.uuid4())[:8])
    status: TaskStatus = TaskStatus.PENDING

    def __lt__(self, other):
        return self.priority < other.priority
