"""
Visualization service for the Model Hub.

This module provides a service for visualizing model performance metrics.
"""

import os
import logging
import json
import tempfile
import base64
from typing import Dict, List, Any, Optional, Tuple, Union
from datetime import datetime, timedelta
import io

# Setup logging
logger = logging.getLogger(__name__)

class VisualizationService:
    """Service for visualizing model performance metrics."""
    
    def __init__(self):
        """Initialize the visualization service."""
        # Check if matplotlib is available
        try:
            import matplotlib
            matplotlib.use("Agg")  # Use non-interactive backend
            import matplotlib.pyplot as plt
            self.matplotlib_available = True
        except ImportError:
            logger.warning("matplotlib not available, visualizations will be limited")
            self.matplotlib_available = False
        
        # Check if plotly is available
        try:
            import plotly.graph_objects as go
            import plotly.express as px
            self.plotly_available = True
        except ImportError:
            logger.warning("plotly not available, visualizations will be limited")
            self.plotly_available = False
        
        logger.info("Visualization service initialized")
    
    async def create_performance_chart(
        self,
        metrics: Dict[str, List[float]],
        timestamps: Optional[List[str]] = None,
        title: str = "Model Performance",
        chart_type: str = "line",
        width: int = 800,
        height: int = 500,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Create a performance chart.
        
        Args:
            metrics: Dictionary of metrics (metric_name -> list of values)
            timestamps: List of timestamps for the metrics
            title: Chart title
            chart_type: Type of chart (line, bar, scatter)
            width: Chart width
            height: Chart height
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (chart data, content type)
        """
        try:
            if self.matplotlib_available:
                return await self._create_matplotlib_chart(
                    metrics=metrics,
                    timestamps=timestamps,
                    title=title,
                    chart_type=chart_type,
                    width=width,
                    height=height,
                    format=format,
                )
            elif self.plotly_available:
                return await self._create_plotly_chart(
                    metrics=metrics,
                    timestamps=timestamps,
                    title=title,
                    chart_type=chart_type,
                    width=width,
                    height=height,
                    format=format,
                )
            else:
                logger.error("No visualization library available")
                raise ValueError("No visualization library available")
        except Exception as e:
            logger.error(f"Error creating performance chart: {e}")
            raise
    
    async def _create_matplotlib_chart(
        self,
        metrics: Dict[str, List[float]],
        timestamps: Optional[List[str]] = None,
        title: str = "Model Performance",
        chart_type: str = "line",
        width: int = 800,
        height: int = 500,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Create a performance chart using matplotlib.
        
        Args:
            metrics: Dictionary of metrics (metric_name -> list of values)
            timestamps: List of timestamps for the metrics
            title: Chart title
            chart_type: Type of chart (line, bar, scatter)
            width: Chart width
            height: Chart height
            format: Output format (png, svg, pdf)
            
        Returns:
            Tuple of (chart data, content type)
        """
        try:
            import matplotlib.pyplot as plt
            import matplotlib.dates as mdates
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
            
            # Create figure
            fig = Figure(figsize=(width / 100, height / 100), dpi=100)
            canvas = FigureCanvas(fig)
            ax = fig.add_subplot(111)
            
            # Create x-axis values
            if timestamps:
                # Convert timestamps to datetime objects
                x = [datetime.fromisoformat(ts) for ts in timestamps]
                
                # Format x-axis
                ax.xaxis.set_major_formatter(mdates.DateFormatter("%Y-%m-%d %H:%M"))
                fig.autofmt_xdate()
            else:
                # Use indices as x-axis values
                x = list(range(len(next(iter(metrics.values())))))
            
            # Plot metrics
            for metric_name, values in metrics.items():
                if chart_type == "line":
                    ax.plot(x, values, label=metric_name)
                elif chart_type == "bar":
                    ax.bar(x, values, label=metric_name, alpha=0.7)
                elif chart_type == "scatter":
                    ax.scatter(x, values, label=metric_name, alpha=0.7)
                else:
                    logger.warning(f"Unsupported chart type: {chart_type}, using line chart")
                    ax.plot(x, values, label=metric_name)
            
            # Add labels and title
            ax.set_xlabel("Time" if timestamps else "Sample")
            ax.set_ylabel("Value")
            ax.set_title(title)
            ax.legend()
            
            # Add grid
            ax.grid(True, linestyle="--", alpha=0.7)
            
            # Adjust layout
            fig.tight_layout()
            
            # Save chart to bytes
            buf = io.BytesIO()
            fig.savefig(buf, format=format)
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error creating matplotlib chart: {e}")
            raise
    
    async def _create_plotly_chart(
        self,
        metrics: Dict[str, List[float]],
        timestamps: Optional[List[str]] = None,
        title: str = "Model Performance",
        chart_type: str = "line",
        width: int = 800,
        height: int = 500,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Create a performance chart using plotly.
        
        Args:
            metrics: Dictionary of metrics (metric_name -> list of values)
            timestamps: List of timestamps for the metrics
            title: Chart title
            chart_type: Type of chart (line, bar, scatter)
            width: Chart width
            height: Chart height
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (chart data, content type)
        """
        try:
            import plotly.graph_objects as go
            from plotly.subplots import make_subplots
            
            # Create figure
            fig = make_subplots(specs=[[{"secondary_y": True}]])
            
            # Create x-axis values
            if timestamps:
                # Use timestamps as x-axis values
                x = timestamps
            else:
                # Use indices as x-axis values
                x = list(range(len(next(iter(metrics.values())))))
            
            # Plot metrics
            for i, (metric_name, values) in enumerate(metrics.items()):
                # Use secondary y-axis for second metric
                secondary_y = i == 1
                
                if chart_type == "line":
                    fig.add_trace(
                        go.Scatter(
                            x=x,
                            y=values,
                            name=metric_name,
                            mode="lines+markers",
                        ),
                        secondary_y=secondary_y,
                    )
                elif chart_type == "bar":
                    fig.add_trace(
                        go.Bar(
                            x=x,
                            y=values,
                            name=metric_name,
                            opacity=0.7,
                        ),
                        secondary_y=secondary_y,
                    )
                elif chart_type == "scatter":
                    fig.add_trace(
                        go.Scatter(
                            x=x,
                            y=values,
                            name=metric_name,
                            mode="markers",
                            opacity=0.7,
                        ),
                        secondary_y=secondary_y,
                    )
                else:
                    logger.warning(f"Unsupported chart type: {chart_type}, using line chart")
                    fig.add_trace(
                        go.Scatter(
                            x=x,
                            y=values,
                            name=metric_name,
                            mode="lines+markers",
                        ),
                        secondary_y=secondary_y,
                    )
            
            # Update layout
            fig.update_layout(
                title=title,
                xaxis_title="Time" if timestamps else "Sample",
                yaxis_title="Value",
                width=width,
                height=height,
                template="plotly_white",
                hovermode="x unified",
            )
            
            # Add grid
            fig.update_xaxes(showgrid=True, gridwidth=1, gridcolor="lightgray")
            fig.update_yaxes(showgrid=True, gridwidth=1, gridcolor="lightgray")
            
            # Convert to requested format
            if format == "html":
                # Return HTML
                html = fig.to_html(include_plotlyjs="cdn")
                return html.encode("utf-8"), "text/html"
            else:
                # Return image
                img_bytes = fig.to_image(format=format, width=width, height=height)
                return img_bytes, f"image/{format}"
        except Exception as e:
            logger.error(f"Error creating plotly chart: {e}")
            raise
    
    async def create_comparison_chart(
        self,
        metrics_a: Dict[str, List[float]],
        metrics_b: Dict[str, List[float]],
        model_a_name: str = "Model A",
        model_b_name: str = "Model B",
        title: str = "Model Comparison",
        chart_type: str = "bar",
        width: int = 800,
        height: int = 500,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Create a comparison chart for two models.
        
        Args:
            metrics_a: Dictionary of metrics for model A (metric_name -> list of values)
            metrics_b: Dictionary of metrics for model B (metric_name -> list of values)
            model_a_name: Name of model A
            model_b_name: Name of model B
            title: Chart title
            chart_type: Type of chart (bar, radar)
            width: Chart width
            height: Chart height
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (chart data, content type)
        """
        try:
            if self.plotly_available:
                return await self._create_plotly_comparison_chart(
                    metrics_a=metrics_a,
                    metrics_b=metrics_b,
                    model_a_name=model_a_name,
                    model_b_name=model_b_name,
                    title=title,
                    chart_type=chart_type,
                    width=width,
                    height=height,
                    format=format,
                )
            elif self.matplotlib_available:
                return await self._create_matplotlib_comparison_chart(
                    metrics_a=metrics_a,
                    metrics_b=metrics_b,
                    model_a_name=model_a_name,
                    model_b_name=model_b_name,
                    title=title,
                    chart_type=chart_type,
                    width=width,
                    height=height,
                    format=format,
                )
            else:
                logger.error("No visualization library available")
                raise ValueError("No visualization library available")
        except Exception as e:
            logger.error(f"Error creating comparison chart: {e}")
            raise
    
    async def _create_matplotlib_comparison_chart(
        self,
        metrics_a: Dict[str, List[float]],
        metrics_b: Dict[str, List[float]],
        model_a_name: str = "Model A",
        model_b_name: str = "Model B",
        title: str = "Model Comparison",
        chart_type: str = "bar",
        width: int = 800,
        height: int = 500,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Create a comparison chart for two models using matplotlib.
        
        Args:
            metrics_a: Dictionary of metrics for model A (metric_name -> list of values)
            metrics_b: Dictionary of metrics for model B (metric_name -> list of values)
            model_a_name: Name of model A
            model_b_name: Name of model B
            title: Chart title
            chart_type: Type of chart (bar, radar)
            width: Chart width
            height: Chart height
            format: Output format (png, svg, pdf)
            
        Returns:
            Tuple of (chart data, content type)
        """
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
            
            # Create figure
            fig = Figure(figsize=(width / 100, height / 100), dpi=100)
            canvas = FigureCanvas(fig)
            
            if chart_type == "bar":
                # Create bar chart
                ax = fig.add_subplot(111)
                
                # Calculate average values for each metric
                metrics = []
                values_a = []
                values_b = []
                
                for metric_name in metrics_a.keys():
                    if metric_name in metrics_b:
                        metrics.append(metric_name)
                        values_a.append(np.mean(metrics_a[metric_name]))
                        values_b.append(np.mean(metrics_b[metric_name]))
                
                # Create x positions
                x = np.arange(len(metrics))
                width = 0.35
                
                # Create bars
                ax.bar(x - width/2, values_a, width, label=model_a_name)
                ax.bar(x + width/2, values_b, width, label=model_b_name)
                
                # Add labels and title
                ax.set_xlabel("Metric")
                ax.set_ylabel("Value")
                ax.set_title(title)
                ax.set_xticks(x)
                ax.set_xticklabels(metrics)
                ax.legend()
                
                # Add grid
                ax.grid(True, linestyle="--", alpha=0.7)
            elif chart_type == "radar":
                # Create radar chart
                ax = fig.add_subplot(111, polar=True)
                
                # Calculate average values for each metric
                metrics = []
                values_a = []
                values_b = []
                
                for metric_name in metrics_a.keys():
                    if metric_name in metrics_b:
                        metrics.append(metric_name)
                        values_a.append(np.mean(metrics_a[metric_name]))
                        values_b.append(np.mean(metrics_b[metric_name]))
                
                # Number of metrics
                N = len(metrics)
                
                # Create angles for each metric
                angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
                
                # Close the polygon
                values_a.append(values_a[0])
                values_b.append(values_b[0])
                angles.append(angles[0])
                metrics.append(metrics[0])
                
                # Plot metrics
                ax.plot(angles, values_a, "o-", linewidth=2, label=model_a_name)
                ax.plot(angles, values_b, "o-", linewidth=2, label=model_b_name)
                ax.fill(angles, values_a, alpha=0.1)
                ax.fill(angles, values_b, alpha=0.1)
                
                # Add labels
                ax.set_thetagrids(np.degrees(angles[:-1]), metrics[:-1])
                ax.set_title(title)
                ax.legend(loc="upper right", bbox_to_anchor=(0.1, 0.1))
            else:
                logger.warning(f"Unsupported chart type: {chart_type}, using bar chart")
                # Create bar chart (same as above)
                ax = fig.add_subplot(111)
                
                # Calculate average values for each metric
                metrics = []
                values_a = []
                values_b = []
                
                for metric_name in metrics_a.keys():
                    if metric_name in metrics_b:
                        metrics.append(metric_name)
                        values_a.append(np.mean(metrics_a[metric_name]))
                        values_b.append(np.mean(metrics_b[metric_name]))
                
                # Create x positions
                x = np.arange(len(metrics))
                width = 0.35
                
                # Create bars
                ax.bar(x - width/2, values_a, width, label=model_a_name)
                ax.bar(x + width/2, values_b, width, label=model_b_name)
                
                # Add labels and title
                ax.set_xlabel("Metric")
                ax.set_ylabel("Value")
                ax.set_title(title)
                ax.set_xticks(x)
                ax.set_xticklabels(metrics)
                ax.legend()
                
                # Add grid
                ax.grid(True, linestyle="--", alpha=0.7)
            
            # Adjust layout
            fig.tight_layout()
            
            # Save chart to bytes
            buf = io.BytesIO()
            fig.savefig(buf, format=format)
            buf.seek(0)
            
            # Get content type
            content_type = f"image/{format}"
            
            return buf.getvalue(), content_type
        except Exception as e:
            logger.error(f"Error creating matplotlib comparison chart: {e}")
            raise
    
    async def _create_plotly_comparison_chart(
        self,
        metrics_a: Dict[str, List[float]],
        metrics_b: Dict[str, List[float]],
        model_a_name: str = "Model A",
        model_b_name: str = "Model B",
        title: str = "Model Comparison",
        chart_type: str = "bar",
        width: int = 800,
        height: int = 500,
        format: str = "png",
    ) -> Tuple[bytes, str]:
        """
        Create a comparison chart for two models using plotly.
        
        Args:
            metrics_a: Dictionary of metrics for model A (metric_name -> list of values)
            metrics_b: Dictionary of metrics for model B (metric_name -> list of values)
            model_a_name: Name of model A
            model_b_name: Name of model B
            title: Chart title
            chart_type: Type of chart (bar, radar)
            width: Chart width
            height: Chart height
            format: Output format (png, svg, pdf, html)
            
        Returns:
            Tuple of (chart data, content type)
        """
        try:
            import plotly.graph_objects as go
            import numpy as np
            
            if chart_type == "bar":
                # Create bar chart
                fig = go.Figure()
                
                # Calculate average values for each metric
                metrics = []
                values_a = []
                values_b = []
                
                for metric_name in metrics_a.keys():
                    if metric_name in metrics_b:
                        metrics.append(metric_name)
                        values_a.append(np.mean(metrics_a[metric_name]))
                        values_b.append(np.mean(metrics_b[metric_name]))
                
                # Add bars
                fig.add_trace(go.Bar(
                    x=metrics,
                    y=values_a,
                    name=model_a_name,
                    opacity=0.7,
                ))
                
                fig.add_trace(go.Bar(
                    x=metrics,
                    y=values_b,
                    name=model_b_name,
                    opacity=0.7,
                ))
                
                # Update layout
                fig.update_layout(
                    title=title,
                    xaxis_title="Metric",
                    yaxis_title="Value",
                    width=width,
                    height=height,
                    template="plotly_white",
                    barmode="group",
                )
            elif chart_type == "radar":
                # Create radar chart
                fig = go.Figure()
                
                # Calculate average values for each metric
                metrics = []
                values_a = []
                values_b = []
                
                for metric_name in metrics_a.keys():
                    if metric_name in metrics_b:
                        metrics.append(metric_name)
                        values_a.append(np.mean(metrics_a[metric_name]))
                        values_b.append(np.mean(metrics_b[metric_name]))
                
                # Add radar chart
                fig.add_trace(go.Scatterpolar(
                    r=values_a + [values_a[0]],
                    theta=metrics + [metrics[0]],
                    fill="toself",
                    name=model_a_name,
                ))
                
                fig.add_trace(go.Scatterpolar(
                    r=values_b + [values_b[0]],
                    theta=metrics + [metrics[0]],
                    fill="toself",
                    name=model_b_name,
                ))
                
                # Update layout
                fig.update_layout(
                    title=title,
                    polar=dict(
                        radialaxis=dict(
                            visible=True,
                        ),
                    ),
                    width=width,
                    height=height,
                    template="plotly_white",
                )
            else:
                logger.warning(f"Unsupported chart type: {chart_type}, using bar chart")
                # Create bar chart (same as above)
                fig = go.Figure()
                
                # Calculate average values for each metric
                metrics = []
                values_a = []
                values_b = []
                
                for metric_name in metrics_a.keys():
                    if metric_name in metrics_b:
                        metrics.append(metric_name)
                        values_a.append(np.mean(metrics_a[metric_name]))
                        values_b.append(np.mean(metrics_b[metric_name]))
                
                # Add bars
                fig.add_trace(go.Bar(
                    x=metrics,
                    y=values_a,
                    name=model_a_name,
                    opacity=0.7,
                ))
                
                fig.add_trace(go.Bar(
                    x=metrics,
                    y=values_b,
                    name=model_b_name,
                    opacity=0.7,
                ))
                
                # Update layout
                fig.update_layout(
                    title=title,
                    xaxis_title="Metric",
                    yaxis_title="Value",
                    width=width,
                    height=height,
                    template="plotly_white",
                    barmode="group",
                )
            
            # Convert to requested format
            if format == "html":
                # Return HTML
                html = fig.to_html(include_plotlyjs="cdn")
                return html.encode("utf-8"), "text/html"
            else:
                # Return image
                img_bytes = fig.to_image(format=format, width=width, height=height)
                return img_bytes, f"image/{format}"
        except Exception as e:
            logger.error(f"Error creating plotly comparison chart: {e}")
            raise
