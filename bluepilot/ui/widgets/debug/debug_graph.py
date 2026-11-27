"""
Reusable Time-Series Graph Widget for BluePilot Debug Panels.
Draws configurable line graphs with grid, scale, time markers, and legend.
"""

import collections
import pyray as rl
from dataclasses import dataclass
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from bluepilot.ui.widgets.debug.debug_colors import DebugColors


@dataclass
class GraphSeries:
  """Definition of a single data series on the graph."""
  label: str
  color: rl.Color
  line_width: float = 3.0
  fill_alpha: int = 0  # 0 = no fill, >0 = semi-transparent fill under curve
  # Bead small circles along the line, spaced every MARKER_STRIDE points. This gives the
  # series a shape distinct from a plain line -- e.g. so a yellow/green pair stays visually
  # distinguishable for red-green colorblind users without changing either color.
  beaded: bool = False


@dataclass
class GraphConfig:
  """Configuration for a TimeSeriesGraph."""
  title: str
  y_unit: str = ""  # e.g. "\u00b0" for degrees, "m/s\u00b2" for accel
  max_data_points: int = 100
  points_per_second: int = 20
  auto_scale: bool = True
  min_scale: float = 5.0
  scale_damping: float = 0.99
  zero_at_bottom: bool = False  # True for 0-1 range graphs (gas/brake)
  max_time_seconds: int = 5  # How many seconds of time labels to show
  title_font_size: int = 52
  axis_font_size: int = 40  # x-axis time labels and y-axis scale labels
  legend_y_offset: int = 0  # shift legend text up (negative) without resizing the plot area
  marker_radius: float = 9.0  # bead size for GraphSeries.beaded (TICI default; MICI uses a smaller value)


class TimeSeriesGraph(Widget):
  """Reusable time-series graph widget for debug panels."""

  # Layout constants
  TOP_MARGIN = 58
  BOTTOM_MARGIN = 15
  SIDE_MARGIN = 85
  TIME_LABELS_HEIGHT = 46
  LEGEND_ROW_HEIGHT = 26
  LEGEND_GAP = 8
  MIN_GRAPH_HEIGHT = 80
  MARKER_STRIDE = 5   # draw a bead every Nth point

  def __init__(self, config: GraphConfig, series: list[GraphSeries]):
    super().__init__()
    self._config = config
    self._series = series
    self._data: list[collections.deque] = [
      collections.deque(maxlen=config.max_data_points) for _ in series
    ]
    self._max_scale = config.min_scale
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_semi = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_normal = gui_app.font(FontWeight.NORMAL)
    self._current_values: list[float] = [0.0] * len(series)
    self._extra_legend_items: list[tuple[str, str]] = []

  def push_data(self, values: list[float]):
    """Push a new data point for each series."""
    for i, val in enumerate(values):
      if i < len(self._data):
        self._data[i].appendleft(val)
        self._current_values[i] = val
    if self._config.auto_scale:
      self._update_scale()

  def set_extra_legend(self, items: list[tuple[str, str]]):
    """Set additional label/value pairs shown in the legend (no plot line)."""
    self._extra_legend_items = items

  def _update_scale(self):
    """Auto-adjust Y scale with damping (matches Qt logic)."""
    for data in self._data:
      for val in data:
        abs_val = abs(val)
        if abs_val > self._max_scale:
          self._max_scale = abs_val * 1.2

    # Slowly shrink scale if data is much smaller
    if all(len(d) > 10 for d in self._data):
      current_max = 0.0
      for data in self._data:
        for val in data:
          current_max = max(current_max, abs(val))
      if current_max < self._max_scale * 0.7:
        self._max_scale *= self._config.scale_damping

    self._max_scale = max(self._max_scale, self._config.min_scale)

  def _render(self, rect: rl.Rectangle):
    # Calculate layout regions
    total_legend_items = len(self._series) + len(self._extra_legend_items)
    legend_rows = max(1, (total_legend_items + 2) // 3)  # 3 columns
    legend_h = legend_rows * self.LEGEND_ROW_HEIGHT + self.LEGEND_GAP
    graph_h = int(rect.height - self.TOP_MARGIN - self.BOTTOM_MARGIN - self.TIME_LABELS_HEIGHT - legend_h)
    graph_h = max(self.MIN_GRAPH_HEIGHT, graph_h)

    graph_x = int(rect.x + self.SIDE_MARGIN)
    graph_w = int(rect.width - 2 * self.SIDE_MARGIN)
    graph_y = int(rect.y + self.TOP_MARGIN)
    time_labels_y = graph_y + graph_h + 12
    legend_y = time_labels_y + self.TIME_LABELS_HEIGHT + self.LEGEND_GAP + self._config.legend_y_offset

    # Draw container background
    self._draw_container(rect)

    # Draw title
    rl.draw_text_ex(self._font_bold, self._config.title,
                    rl.Vector2(graph_x, rect.y + 8), self._config.title_font_size, 0,
                    DebugColors.SCALE_TEXT)

    # Draw graph area background
    rl.draw_rectangle(graph_x, graph_y, graph_w, graph_h, DebugColors.GRAPH_BG)
    rl.draw_rectangle_lines(graph_x, graph_y, graph_w, graph_h, DebugColors.GRAPH_BORDER)

    # Clip graph content
    rl.begin_scissor_mode(graph_x, graph_y, graph_w, graph_h)

    # Draw grid lines
    self._draw_grid(graph_x, graph_y, graph_w, graph_h)

    # Draw zero line
    if self._config.zero_at_bottom:
      zero_y = graph_y + graph_h  # Zero at bottom
    else:
      zero_y = graph_y + graph_h // 2  # Zero centered
    rl.draw_line_ex(rl.Vector2(graph_x, zero_y),
                    rl.Vector2(graph_x + graph_w, zero_y),
                    2.0, DebugColors.ZERO_LINE)

    # Draw data series
    for i, series in enumerate(self._series):
      self._draw_series(i, series, graph_x, graph_y, graph_w, graph_h, zero_y)

    rl.end_scissor_mode()

    # Draw time markers (below graph)
    self._draw_time_markers(graph_x, graph_y, graph_w, graph_h, time_labels_y)

    # Draw scale labels (left of graph)
    self._draw_scale(rect, graph_x, graph_y, graph_h, zero_y)

    # Draw legend
    self._draw_legend(graph_x, graph_w, legend_y, rect)

  def _draw_container(self, rect: rl.Rectangle):
    """Draw the rounded container background with metallic styling."""
    container = rl.Rectangle(rect.x + 8, rect.y + 8, rect.width - 16, rect.height - 16)
    rl.draw_rectangle_rounded(container, 0.04, 10, DebugColors.GRAPH_CONTAINER_MID)
    rl.draw_rectangle_rounded_lines_ex(container, 0.04, 10, 1.5,
                                       rl.Color(70, 130, 180, 120))

  def _draw_grid(self, gx: int, gy: int, gw: int, gh: int):
    """Draw horizontal grid lines."""
    divisions = 8 if not self._config.zero_at_bottom else 4
    center = divisions // 2

    for i in range(1, divisions):
      if not self._config.zero_at_bottom and i == center:
        continue  # Skip center (zero line drawn separately)
      y = gy + (i * gh // divisions)
      # Draw dotted-style grid by drawing short dashes
      x = gx
      while x < gx + gw:
        dash_len = min(6, gx + gw - x)
        rl.draw_line_ex(rl.Vector2(x, y), rl.Vector2(x + dash_len, y),
                        1.0, DebugColors.GRID_LINE)
        x += 12  # dash + gap

  def _draw_time_markers(self, gx: int, gy: int, gw: int, gh: int, labels_y: int):
    """Draw vertical time markers and labels."""
    max_points = self._config.max_data_points
    # Find longest data series length for spacing calculation
    max_len = max((len(d) for d in self._data), default=1)
    max_len = max(1, max_len)
    point_spacing = gw / min(max_points, max_len)
    line_spacing = self._config.points_per_second * point_spacing

    for i in range(self._config.max_time_seconds + 1):
      x = gx + gw - int(i * line_spacing)
      if x < gx:
        break

      # Vertical grid line (dotted)
      color = DebugColors.TIME_MARKER_PRIMARY if i <= 5 else DebugColors.TIME_MARKER_SECONDARY
      y = gy
      while y < gy + gh:
        dash_len = min(6, gy + gh - y)
        rl.draw_line_ex(rl.Vector2(x, y), rl.Vector2(x, y + dash_len), 1.0, color)
        y += 12

      # Time label
      label = "Now" if i == 0 else f"-{i}s"
      rl.draw_text_ex(self._font_normal, label,
                      rl.Vector2(x - 10, labels_y), self._config.axis_font_size, 0, DebugColors.LEGEND_TEXT)

  def _draw_scale(self, rect: rl.Rectangle, gx: int, gy: int, gh: int, zero_y: int):
    """Draw Y-axis scale labels."""
    scale_x = int(rect.x + 4)
    font_size = self._config.axis_font_size

    if self._config.zero_at_bottom:
      # 0-1 range
      rl.draw_text_ex(self._font_semi, "0.0",
                      rl.Vector2(scale_x, gy + gh - 14), font_size, 0, DebugColors.SCALE_TEXT)
      rl.draw_text_ex(self._font_semi, "1.0",
                      rl.Vector2(scale_x, gy + 2), font_size, 0, DebugColors.SCALE_TEXT)
      rl.draw_text_ex(self._font_semi, "0.5",
                      rl.Vector2(scale_x, gy + gh // 2 - 6), font_size, 0, DebugColors.SCALE_TEXT)
    else:
      # Centered scale
      unit = self._config.y_unit
      rl.draw_text_ex(self._font_semi, f"0{unit}",
                      rl.Vector2(scale_x, zero_y - 6), font_size, 0, DebugColors.SCALE_TEXT)
      rl.draw_text_ex(self._font_semi, f"+{self._max_scale:.0f}{unit}",
                      rl.Vector2(scale_x, gy + 2), font_size, 0, DebugColors.SCALE_TEXT)
      rl.draw_text_ex(self._font_semi, f"-{self._max_scale:.0f}{unit}",
                      rl.Vector2(scale_x, gy + gh - 14), font_size, 0, DebugColors.SCALE_TEXT)

  def _draw_series(self, series_idx: int, series: GraphSeries,
                   gx: int, gy: int, gw: int, gh: int, zero_y: int):
    """Draw a single data series as connected line segments with optional fill."""
    data = self._data[series_idx]
    if len(data) < 2:
      return

    max_len = max((len(d) for d in self._data), default=1)
    max_len = max(1, max_len)
    point_spacing = gw / min(self._config.max_data_points, max_len)

    # Pre-compute Y positions
    points = []
    for i in range(len(data)):
      x = gx + gw - i * point_spacing
      if x < gx:
        break

      if self._config.zero_at_bottom:
        # 0-1 range: 0 at bottom, 1 at top
        y = gy + gh - (data[i] * gh)
      else:
        # Centered: value / max_scale maps to half-height
        y = zero_y - (data[i] / self._max_scale) * (gh / 2)

      y = max(gy, min(y, gy + gh))
      points.append((x, y))

    if len(points) < 2:
      return

    # Draw filled area under curve (optional)
    if series.fill_alpha > 0:
      fill_color = rl.Color(series.color.r, series.color.g, series.color.b, series.fill_alpha)
      base_y = float(zero_y if not self._config.zero_at_bottom else gy + gh)
      for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        # Two triangles to fill the area between line and base
        rl.draw_triangle(rl.Vector2(x1, y1), rl.Vector2(x2, base_y),
                         rl.Vector2(x2, y2), fill_color)
        rl.draw_triangle(rl.Vector2(x1, y1), rl.Vector2(x1, base_y),
                         rl.Vector2(x2, base_y), fill_color)

    # Draw line segments
    for i in range(len(points) - 1):
      x1, y1 = points[i]
      x2, y2 = points[i + 1]
      rl.draw_line_ex(rl.Vector2(x1, y1), rl.Vector2(x2, y2),
                      series.line_width, series.color)

    # Bead small circles along the line so this series reads as a distinct shape,
    # not just a distinct color (see GraphSeries.beaded)
    if series.beaded:
      for i in range(0, len(points), self.MARKER_STRIDE):
        x, y = points[i]
        rl.draw_circle(int(x), int(y), self._config.marker_radius, series.color)

  def _draw_legend(self, gx: int, gw: int, legend_y: int, rect: rl.Rectangle):
    """Draw the legend with color swatches, labels, and current values."""
    font_size = 16
    col_width = gw // 3

    all_items = []
    # Series items with color swatch
    for i, series in enumerate(self._series):
      val = self._current_values[i] if i < len(self._current_values) else 0.0
      unit = self._config.y_unit
      if self._config.zero_at_bottom:
        text = f"{series.label}: {val:.3f}"
      else:
        text = f"{series.label}: {val:.1f}{unit}"
      all_items.append((series.color, text))

    # Extra legend items (no color swatch)
    for label, value in self._extra_legend_items:
      all_items.append((None, f"{label}: {value}"))

    for idx, (color, text) in enumerate(all_items):
      col = idx % 3
      row = idx // 3
      x = gx + col * col_width
      y = legend_y + row * self.LEGEND_ROW_HEIGHT

      if color is not None:
        # Draw color swatch
        rl.draw_rectangle(x, y, 16, 16, color)
        rl.draw_text_ex(self._font_normal, text,
                        rl.Vector2(x + 22, y - 2), font_size, 0, DebugColors.LEGEND_TEXT)
      else:
        rl.draw_text_ex(self._font_normal, text,
                        rl.Vector2(x + 22, y - 2), font_size, 0, DebugColors.LEGEND_LABEL)
