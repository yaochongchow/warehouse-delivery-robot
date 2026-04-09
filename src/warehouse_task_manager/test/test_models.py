"""Tests for warehouse_task_manager.models."""

import heapq
import pytest

from warehouse_task_manager.models import DeliveryOrder, TaskStatus


class TestTaskStatus:
    def test_all_values_exist(self):
        expected = {
            'pending', 'in_progress', 'navigating_to_pickup', 'picking_up',
            'navigating_to_delivery', 'dropping_off', 'completed',
            'cancelled', 'failed',
        }
        actual = {s.value for s in TaskStatus}
        assert actual == expected

    def test_enum_access(self):
        assert TaskStatus.PENDING.value == 'pending'
        assert TaskStatus.COMPLETED.value == 'completed'
        assert TaskStatus.FAILED.value == 'failed'

    def test_enum_identity(self):
        assert TaskStatus('pending') is TaskStatus.PENDING


class TestDeliveryOrder:
    def test_default_fields(self):
        order = DeliveryOrder(pickup_station='A', delivery_station='D1')
        assert order.pickup_station == 'A'
        assert order.delivery_station == 'D1'
        assert order.priority == 1
        assert order.status == TaskStatus.PENDING
        assert len(order.order_id) == 8

    def test_unique_ids(self):
        o1 = DeliveryOrder(pickup_station='A', delivery_station='D1')
        o2 = DeliveryOrder(pickup_station='A', delivery_station='D1')
        assert o1.order_id != o2.order_id

    def test_custom_priority(self):
        order = DeliveryOrder(pickup_station='A', delivery_station='D1', priority=5)
        assert order.priority == 5

    def test_ordering_lower_priority_first(self):
        high = DeliveryOrder(pickup_station='A', delivery_station='D1', priority=1)
        low = DeliveryOrder(pickup_station='B', delivery_station='D2', priority=5)
        assert high < low

    def test_ordering_equal_priority(self):
        o1 = DeliveryOrder(pickup_station='A', delivery_station='D1', priority=3)
        o2 = DeliveryOrder(pickup_station='B', delivery_station='D2', priority=3)
        # Not less than when equal
        assert not (o1 < o2)
        assert not (o2 < o1)

    def test_heapq_ordering(self):
        """Priority queue should pop lowest-priority-number first."""
        orders = [
            DeliveryOrder(pickup_station='A', delivery_station='D1', priority=3),
            DeliveryOrder(pickup_station='B', delivery_station='D2', priority=1),
            DeliveryOrder(pickup_station='A', delivery_station='D2', priority=2),
        ]
        heap = []
        for o in orders:
            heapq.heappush(heap, (o.priority, o))

        _, first = heapq.heappop(heap)
        _, second = heapq.heappop(heap)
        _, third = heapq.heappop(heap)

        assert first.priority == 1
        assert second.priority == 2
        assert third.priority == 3

    def test_status_mutation(self):
        order = DeliveryOrder(pickup_station='A', delivery_station='D1')
        assert order.status == TaskStatus.PENDING
        order.status = TaskStatus.IN_PROGRESS
        assert order.status == TaskStatus.IN_PROGRESS
        order.status = TaskStatus.COMPLETED
        assert order.status == TaskStatus.COMPLETED
