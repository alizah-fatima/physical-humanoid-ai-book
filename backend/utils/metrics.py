"""
Metrics and performance monitoring for RAG Agent
"""
import time
from typing import Dict, Any, Optional, Callable
from datetime import datetime
from dataclasses import dataclass
from enum import Enum


class MetricType(Enum):
    """Types of metrics that can be collected"""
    COUNTER = "counter"
    GAUGE = "gauge"
    HISTOGRAM = "histogram"
    SUMMARY = "summary"


@dataclass
class Metric:
    """Data class for a metric"""
    name: str
    type: MetricType
    value: float
    labels: Dict[str, str]
    timestamp: datetime


class MetricsCollector:
    """Collects and manages metrics for the RAG Agent"""

    def __init__(self):
        self.metrics: Dict[str, Metric] = {}
        self.start_times: Dict[str, float] = {}

    def start_timer(self, operation: str):
        """Start timing an operation"""
        self.start_times[operation] = time.time()

    def stop_timer(self, operation: str) -> float:
        """Stop timing an operation and return elapsed time in seconds"""
        if operation in self.start_times:
            elapsed = time.time() - self.start_times[operation]
            del self.start_times[operation]

            # Record the timing metric
            self.record_histogram(f"{operation}_duration_seconds", elapsed)
            return elapsed
        return 0.0

    def record_counter(self, name: str, value: float = 1.0, labels: Optional[Dict[str, str]] = None):
        """Record a counter metric"""
        labels = labels or {}
        metric_key = f"{name}_{str(labels)}"

        if metric_key in self.metrics:
            self.metrics[metric_key].value += value
        else:
            self.metrics[metric_key] = Metric(
                name=name,
                type=MetricType.COUNTER,
                value=value,
                labels=labels,
                timestamp=datetime.now()
            )

    def record_gauge(self, name: str, value: float, labels: Optional[Dict[str, str]] = None):
        """Record a gauge metric"""
        labels = labels or {}
        metric_key = f"{name}_{str(labels)}"

        self.metrics[metric_key] = Metric(
            name=name,
            type=MetricType.GAUGE,
            value=value,
            labels=labels,
            timestamp=datetime.now()
        )

    def record_histogram(self, name: str, value: float, labels: Optional[Dict[str, str]] = None):
        """Record a histogram metric"""
        labels = labels or {}
        metric_key = f"{name}_{str(labels)}"

        # For simplicity, we'll just store the latest value
        # In a real implementation, we'd store multiple values for histogram analysis
        self.metrics[metric_key] = Metric(
            name=name,
            type=MetricType.HISTOGRAM,
            value=value,
            labels=labels,
            timestamp=datetime.now()
        )

    def record_summary(self, name: str, value: float, labels: Optional[Dict[str, str]] = None):
        """Record a summary metric"""
        labels = labels or {}
        metric_key = f"{name}_{str(labels)}"

        self.metrics[metric_key] = Metric(
            name=name,
            type=MetricType.SUMMARY,
            value=value,
            labels=labels,
            timestamp=datetime.now()
        )

    def get_metric(self, name: str, labels: Optional[Dict[str, str]] = None) -> Optional[Metric]:
        """Get a specific metric by name and labels"""
        labels = labels or {}
        metric_key = f"{name}_{str(labels)}"
        return self.metrics.get(metric_key)

    def get_all_metrics(self) -> Dict[str, Metric]:
        """Get all collected metrics"""
        return self.metrics.copy()

    def reset_metrics(self):
        """Reset all metrics"""
        self.metrics.clear()


class PerformanceMonitor:
    """Monitors performance of the RAG Agent"""

    def __init__(self):
        self.collector = MetricsCollector()

    def measure_query_performance(self, query_func: Callable, *args, **kwargs) -> Any:
        """Measure performance of a query function"""
        operation = "query_processing"
        self.collector.start_timer(operation)

        try:
            result = query_func(*args, **kwargs)
            self.collector.record_counter("queries_total", 1.0)
            return result
        except Exception as e:
            self.collector.record_counter("query_errors_total", 1.0)
            raise
        finally:
            duration = self.collector.stop_timer(operation)
            self.collector.record_histogram("query_duration_seconds", duration)

    def measure_retrieval_performance(self, retrieval_func: Callable, *args, **kwargs) -> Any:
        """Measure performance of a retrieval function"""
        operation = "retrieval_processing"
        self.collector.start_timer(operation)

        try:
            result = retrieval_func(*args, **kwargs)
            self.collector.record_counter("retrieval_operations_total", 1.0)
            return result
        except Exception as e:
            self.collector.record_counter("retrieval_errors_total", 1.0)
            raise
        finally:
            duration = self.collector.stop_timer(operation)
            self.collector.record_histogram("retrieval_duration_seconds", duration)

    def track_agent_metrics(self, query_text: str, response_length: int, context_chunks: int):
        """Track various agent metrics"""
        # Track query length
        self.collector.record_histogram("query_length_chars", len(query_text))

        # Track response length
        self.collector.record_histogram("response_length_chars", response_length)

        # Track context usage
        self.collector.record_gauge("context_chunks_used", context_chunks)

        # Track tokens (approximate)
        approx_tokens = response_length // 4  # Rough estimate: 1 token ~ 4 chars
        self.collector.record_histogram("response_tokens_approx", approx_tokens)

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get a summary of performance metrics"""
        all_metrics = self.collector.get_all_metrics()

        summary = {
            "timestamp": datetime.now().isoformat(),
            "total_metrics": len(all_metrics),
            "metrics": {}
        }

        for key, metric in all_metrics.items():
            summary["metrics"][metric.name] = {
                "value": metric.value,
                "type": metric.type.value,
                "labels": metric.labels,
                "timestamp": metric.timestamp.isoformat()
            }

        return summary


# Global performance monitor instance
performance_monitor = PerformanceMonitor()


def get_performance_monitor() -> PerformanceMonitor:
    """Get the global performance monitor instance"""
    return performance_monitor