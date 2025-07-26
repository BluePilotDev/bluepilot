#!/usr/bin/env python3

import pyray as rl
import subprocess
import sys
import os
import threading
import random
import shutil
import time
from datetime import datetime, timedelta
from typing import Optional, List, Dict, Tuple, Callable
from dataclasses import dataclass
from enum import Enum

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.widget import Widget
from openpilot.system.ui.text import wrap_text
from openpilot.common.params import Params

# Design Constants
MARGIN = 40
CARD_PADDING = 30
CARD_SPACING = 20
BUTTON_HEIGHT = 80
BUTTON_SPACING = 15
HEADER_SIZE = 48
TITLE_SIZE = 36
TEXT_SIZE = 28
SMALL_SIZE = 24

# Modern Color Palette
BG_COLOR = (10, 10, 12, 255)           # Dark background
CARD_BG = (24, 24, 28, 255)           # Card background
CARD_HOVER = (32, 32, 36, 255)        # Card hover
PRIMARY = (59, 130, 246, 255)         # Blue
PRIMARY_HOVER = (37, 99, 235, 255)    # Blue hover
SUCCESS = (34, 197, 94, 255)          # Green
SUCCESS_HOVER = (22, 163, 74, 255)    # Green hover
WARNING = (251, 146, 60, 255)         # Orange
WARNING_HOVER = (249, 115, 22, 255)   # Orange hover
DANGER = (239, 68, 68, 255)           # Red
DANGER_HOVER = (220, 38, 38, 255)     # Red hover
TEXT_PRIMARY = (248, 250, 252, 255)   # White
TEXT_SECONDARY = (156, 163, 175, 255) # Gray
TEXT_MUTED = (107, 114, 128, 255)     # Dark gray
ACCENT = (139, 92, 246, 255)          # Purple

class UpdaterStatus(Enum):
    OK = "OK"
    NO_INTERNET = "No Internet Connection"
    SSH_AUTH_FAILED = "SSH Authentication Failed"
    NO_REMOTE_BRANCH = "No Remote Branch"
    ONROAD = "Vehicle in Motion"

@dataclass
class GitStatus:
    clean: bool = True
    has_updates: bool = False
    commit_hash: str = ""
    commit_message: str = ""
    commit_time: str = ""
    status_text: str = "Clean"
    status_color: Tuple[int, int, int, int] = SUCCESS

@dataclass
class SubmoduleInfo:
    name: str
    status: str = "Checking..."
    color: Tuple[int, int, int, int] = TEXT_MUTED
    has_updates: bool = False
    has_changes: bool = False

class Timer:
    def __init__(self, interval_ms: int, callback: Callable):
        self.interval_ms = interval_ms
        self.callback = callback
        self.last_time = 0
        self.running = False

    def start(self):
        self.running = True
        self.last_time = time.time() * 1000

    def stop(self):
        self.running = False

    def update(self):
        if not self.running:
            return
        current_time = time.time() * 1000
        if current_time - self.last_time >= self.interval_ms:
            self.callback()
            self.last_time = current_time

class Button:
    def __init__(self, text: str, color: Tuple[int, int, int, int] = PRIMARY,
                 hover_color: Tuple[int, int, int, int] = PRIMARY_HOVER,
                 enabled: bool = True, visible: bool = True):
        self.rect = rl.Rectangle(0, 0, 0, BUTTON_HEIGHT)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.enabled = enabled
        self.visible = visible
        self.hovered = False
        self.pressed = False
        self.clicked = False

    def update(self, mouse_pos: rl.Vector2, mouse_pressed: bool, mouse_released: bool):
        self.clicked = False
        if not self.visible or not self.enabled:
            self.hovered = self.pressed = False
            return

        self.hovered = rl.check_collision_point_rec(mouse_pos, self.rect)

        if self.hovered and mouse_pressed:
            self.pressed = True
        elif mouse_released and self.pressed and self.hovered:
            self.clicked = True
            self.pressed = False
        elif mouse_released:
            self.pressed = False

    def draw(self, font_size: int = TEXT_SIZE):
        if not self.visible:
            return

        # Determine colors
        if not self.enabled:
            bg_color = (64, 64, 64, 255)
            text_color = TEXT_MUTED
        elif self.pressed:
            bg_color = tuple(max(0, c - 30) for c in self.color[:3]) + (255,)
            text_color = TEXT_PRIMARY
        elif self.hovered:
            bg_color = self.hover_color
            text_color = TEXT_PRIMARY
        else:
            bg_color = self.color
            text_color = TEXT_PRIMARY

        # Draw button with shadow
        shadow_rect = rl.Rectangle(self.rect.x + 2, self.rect.y + 2, self.rect.width, self.rect.height)
        rl.draw_rectangle_rounded(shadow_rect, 0.25, 12, (0, 0, 0, 40))
        rl.draw_rectangle_rounded(self.rect, 0.25, 12, bg_color)

        # Draw text
        text_size = measure_text_cached(gui_app.font(), self.text, font_size)
        text_x = self.rect.x + (self.rect.width - text_size.x) / 2
        text_y = self.rect.y + (self.rect.height - text_size.y) / 2
        rl.draw_text_ex(gui_app.font(), self.text, rl.Vector2(text_x, text_y),
                       font_size, 0.0, text_color)

class Card:
    def __init__(self, x: float, y: float, width: float, height: float):
        self.rect = rl.Rectangle(x, y, width, height)

    def draw_background(self):
        # Shadow
        shadow_rect = rl.Rectangle(self.rect.x + 4, self.rect.y + 4, self.rect.width, self.rect.height)
        rl.draw_rectangle_rounded(shadow_rect, 0.08, 16, (0, 0, 0, 30))

        # Card background
        rl.draw_rectangle_rounded(self.rect, 0.08, 16, CARD_BG)

class ScrollableOutput:
    def __init__(self, x: float, y: float, width: float, height: float):
        self.rect = rl.Rectangle(x, y, width, height)
        self.lines: List[str] = []
        self.scroll_offset = 0
        self.max_scroll = 0
        self.line_height = TEXT_SIZE + 4

    def add_text(self, text: str):
        for line in text.split('\n'):
            wrapped = wrap_text(line, TEXT_SIZE, self.rect.width - 60)
            self.lines.extend(wrapped)

        # Limit history
        if len(self.lines) > 1000:
            self.lines = self.lines[-800:]

        self.max_scroll = max(0, len(self.lines) * self.line_height - self.rect.height)
        self.scroll_offset = self.max_scroll  # Auto-scroll to bottom

    def handle_scroll(self, wheel_move: float):
        self.scroll_offset -= int(wheel_move * 40)
        self.scroll_offset = max(0, min(self.scroll_offset, self.max_scroll))

    def draw(self):
        # Background
        rl.draw_rectangle_rounded(self.rect, 0.08, 16, (16, 16, 20, 255))

        # Content with clipping
        rl.begin_scissor_mode(int(self.rect.x), int(self.rect.y),
                             int(self.rect.width), int(self.rect.height))

        start_line = self.scroll_offset // self.line_height
        visible_lines = int(self.rect.height / self.line_height) + 2

        for i in range(start_line, min(start_line + visible_lines, len(self.lines))):
            y = self.rect.y + (i * self.line_height) - self.scroll_offset + 20
            if y >= self.rect.y - self.line_height and y <= self.rect.y + self.rect.height:
                # Color-code output
                line = self.lines[i]
                color = TEXT_PRIMARY
                if "error" in line.lower() or "failed" in line.lower():
                    color = DANGER
                elif "warning" in line.lower():
                    color = WARNING
                elif "success" in line.lower() or "completed" in line.lower():
                    color = SUCCESS

                rl.draw_text_ex(gui_app.font(), line, rl.Vector2(self.rect.x + 20, y),
                               TEXT_SIZE, 0.0, color)

        rl.end_scissor_mode()

        # Scrollbar
        if self.max_scroll > 0:
            scrollbar_height = max(20, (self.rect.height / (len(self.lines) * self.line_height)) * self.rect.height)
            scrollbar_y = self.rect.y + (self.scroll_offset / self.max_scroll) * (self.rect.height - scrollbar_height)
            rl.draw_rectangle_rounded(rl.Rectangle(self.rect.x + self.rect.width - 8, scrollbar_y, 6, scrollbar_height),
                                     0.3, 8, PRIMARY)

class CommandDialog:
    def __init__(self, title: str, command: str):
        self.title = title
        self.command = command
        self.running = False
        self.process: Optional[subprocess.Popen] = None
        self.start_time = time.time()
        self.output = ScrollableOutput(60, 120, gui_app.width - 120, gui_app.height - 240)

        # Buttons
        self.kill_btn = Button("Stop", DANGER, DANGER_HOVER)
        self.retry_btn = Button("Retry", WARNING, WARNING_HOVER, visible=False)
        self.reboot_btn = Button("Reboot", SUCCESS, SUCCESS_HOVER, visible=False)
        self.close_btn = Button("Running...", PRIMARY, PRIMARY_HOVER, enabled=False)

    def start(self):
        if self.running:
            return

        self.running = True
        self.start_time = time.time()
        self.output.add_text(f"Executing: {self.command}\n")

        # Kill any existing update processes
        try:
            subprocess.run(["killall", "system.updated.updated"], capture_output=True, timeout=5)
        except:
            pass

        def run_command():
            try:
                self.process = subprocess.Popen(
                    ["/bin/bash", "-c", self.command],
                    cwd="/data/openpilot",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1
                )

                while True:
                    line = self.process.stdout.readline()
                    if not line and self.process.poll() is not None:
                        break
                    if line:
                        self.output.add_text(line.rstrip())

                self.process.wait()
                success = self.process.returncode == 0

                if success:
                    self.output.add_text("\n✓ Command completed successfully!")
                    self.reboot_btn.visible = True
                else:
                    self.output.add_text(f"\n✗ Command failed (exit code: {self.process.returncode})")
                    self.retry_btn.visible = True

            except Exception as e:
                self.output.add_text(f"\n✗ Error: {str(e)}")
                self.retry_btn.visible = True

            finally:
                self.running = False
                self.kill_btn.visible = False
                self.close_btn.enabled = True
                self.close_btn.text = "Close"

        threading.Thread(target=run_command, daemon=True).start()

    def update(self, mouse_pos: rl.Vector2, mouse_pressed: bool, mouse_released: bool, wheel_move: float):
        # Update buttons
        self.kill_btn.update(mouse_pos, mouse_pressed, mouse_released)
        self.retry_btn.update(mouse_pos, mouse_pressed, mouse_released)
        self.reboot_btn.update(mouse_pos, mouse_pressed, mouse_released)
        self.close_btn.update(mouse_pos, mouse_pressed, mouse_released)

        # Handle scrolling
        self.output.handle_scroll(wheel_move)

        # Handle clicks
        if self.kill_btn.clicked and self.process:
            try:
                self.process.kill()
                self.output.add_text("\n⚠ Process terminated by user")
                self.running = False
                self.kill_btn.visible = False
                self.retry_btn.visible = True
                self.close_btn.enabled = True
                self.close_btn.text = "Close"
            except:
                pass

        if self.retry_btn.clicked:
            self.retry_btn.visible = False
            self.reboot_btn.visible = False
            self.close_btn.enabled = False
            self.close_btn.text = "Running..."
            self.kill_btn.visible = True
            self.start()

        if self.reboot_btn.clicked:
            try:
                subprocess.run(["sudo", "reboot"], check=False)
            except:
                subprocess.run(["reboot"], check=False)

        return self.close_btn.clicked

    def draw(self):
        # Background
        rl.draw_rectangle(0, 0, gui_app.width, gui_app.height, BG_COLOR)

        # Title
        title_size = measure_text_cached(gui_app.font(), self.title, HEADER_SIZE)
        title_x = (gui_app.width - title_size.x) / 2
        rl.draw_text_ex(gui_app.font(), self.title, rl.Vector2(title_x, 40),
                       HEADER_SIZE, 0.0, TEXT_PRIMARY)

        # Output
        self.output.draw()

        # Runtime indicator
        if self.running:
            elapsed = int(time.time() - self.start_time)
            runtime = f"Running: {elapsed//60:02d}:{elapsed%60:02d}"
            self.close_btn.text = runtime

        # Buttons
        button_y = gui_app.height - 100
        buttons = [b for b in [self.kill_btn, self.retry_btn, self.reboot_btn, self.close_btn] if b.visible]

        if buttons:
            button_width = 160
            total_width = len(buttons) * button_width + (len(buttons) - 1) * 20
            start_x = (gui_app.width - total_width) / 2

            for i, btn in enumerate(buttons):
                btn.rect.x = start_x + i * (button_width + 20)
                btn.rect.y = button_y
                btn.rect.width = button_width
                btn.draw()

class BPUpdaterPanel(Widget):
    def __init__(self, from_spinner: bool = False):
        super().__init__()
        self.params = Params()
        self.from_spinner = from_spinner

        # State
        self.git_status = GitStatus()
        self.submodules: List[SubmoduleInfo] = []
        self.current_branch = "main"
        self.current_repo = "bluepilot/openpilot"
        self.updater_status = UpdaterStatus.OK
        self.last_update_check: Optional[datetime] = None
        self.command_dialog: Optional[CommandDialog] = None

        # Buttons
        self.buttons: Dict[str, Button] = {}
        self.submodule_buttons: Dict[str, Dict[str, Button]] = {}

        # Timers
        self.auto_update_timer = Timer(1800000, self.check_for_updates)  # 30 min
        self.refresh_timer = Timer(60000, self.refresh_data)  # 1 min
        self.param_timer = Timer(1000, self.check_onroad)  # 1 sec

        self.init_buttons()
        self.start_timers()
        self.refresh_data()

    def init_buttons(self):
        self.buttons = {
            'check_updates': Button("Check Updates", PRIMARY, PRIMARY_HOVER),
            'update_repo': Button("Update", SUCCESS, SUCCESS_HOVER, visible=False),
            'update_all': Button("Update All", SUCCESS, SUCCESS_HOVER),
            'repair_repo': Button("Repair", ACCENT, tuple(max(0, c - 30) for c in ACCENT[:3]) + (255,)),
            'reset_repo': Button("Reset", DANGER, DANGER_HOVER, visible=False),
            'show_commits': Button("History", WARNING, WARNING_HOVER),
            'change_branch': Button("Change Branch", PRIMARY, PRIMARY_HOVER),
        }

        if self.from_spinner:
            self.buttons['close_panel'] = Button("Close", TEXT_MUTED, (128, 128, 128, 255))

    def start_timers(self):
        self.auto_update_timer.start()
        self.refresh_timer.start()
        self.param_timer.start()

    def stop_timers(self):
        self.auto_update_timer.stop()
        self.refresh_timer.stop()
        self.param_timer.stop()

    def execute_git_command(self, command: str, working_dir: str = "/data/openpilot", timeout: float = 30.0):
        try:
            result = subprocess.run(["/bin/bash", "-c", command], cwd=working_dir,
                                   capture_output=True, text=True, timeout=timeout)
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
        except Exception as e:
            return False, "", str(e)

    def is_onroad(self) -> bool:
        try:
            return self.params.get_bool("IsOnroad")
        except:
            return False

    def check_onroad(self):
        if self.is_onroad():
            self.updater_status = UpdaterStatus.ONROAD
            self.auto_update_timer.stop()
        else:
            if self.updater_status == UpdaterStatus.ONROAD:
                self.updater_status = UpdaterStatus.OK
                self.auto_update_timer.start()

    def check_internet(self) -> bool:
        success, _, _ = self.execute_git_command("curl -sSfI --max-time 5 https://github.com", timeout=10)
        return success

    def check_ssh_valid(self) -> bool:
        if not os.path.exists("/home/comma/.ssh/github"):
            return False
        success, _, error = self.execute_git_command('ssh -T -o BatchMode=yes -o ConnectTimeout=5 git@github.com', timeout=10)
        return "successfully authenticated" in error

    def refresh_git_status(self):
        # Get branch and repo info
        success, output, _ = self.execute_git_command("git branch --show-current")
        if success and output.strip():
            self.current_branch = output.strip()

        success, output, _ = self.execute_git_command("git remote get-url origin")
        if success:
            url = output.strip()
            if "git@" in url:
                self.current_repo = url.split(":")[1].replace(".git", "")
            elif "https://" in url:
                self.current_repo = "/".join(url.split("/")[-2:]).replace(".git", "")

        # Check status
        success, output, _ = self.execute_git_command("git status --porcelain")
        clean = success and not output.strip()

        # Check updates
        self.execute_git_command("git fetch", timeout=45)
        success, output, _ = self.execute_git_command("git rev-list HEAD..@{u} --count")
        has_updates = success and int(output.strip() or "0") > 0

        # Get commit info
        success, output, _ = self.execute_git_command("git log -1 --pretty=format:'%h|||%s|||%cr'")
        if success and "|||" in output:
            parts = output.split("|||")
            if len(parts) == 3:
                self.git_status.commit_hash, self.git_status.commit_message, self.git_status.commit_time = parts

        # Set status
        if not clean and has_updates:
            self.git_status.status_text = "Updates Available (Modified)"
            self.git_status.status_color = WARNING
        elif has_updates:
            self.git_status.status_text = "Updates Available"
            self.git_status.status_color = PRIMARY
        elif not clean:
            self.git_status.status_text = "Modified"
            self.git_status.status_color = WARNING
        else:
            self.git_status.status_text = "Up to Date"
            self.git_status.status_color = SUCCESS

        self.git_status.clean = clean
        self.git_status.has_updates = has_updates

    def refresh_submodules(self):
        success, output, _ = self.execute_git_command("git submodule status")
        if not success or not output.strip():
            self.submodules = []
            return

        new_submodules = []
        for line in output.strip().split('\n'):
            if line.strip():
                parts = line.strip().split()
                if len(parts) >= 2:
                    name = parts[1]
                    submodule = SubmoduleInfo(name=name)
                    new_submodules.append(submodule)

                    # Init buttons
                    if name not in self.submodule_buttons:
                        self.submodule_buttons[name] = {
                            'update': Button("Update", SUCCESS, SUCCESS_HOVER, visible=False),
                            'reset': Button("Reset", DANGER, DANGER_HOVER, visible=False),
                            'repair': Button("Repair", ACCENT, tuple(max(0, c - 30) for c in ACCENT[:3]) + (255,)),
                        }

        self.submodules = new_submodules

        # Check each submodule
        for submodule in self.submodules:
            self.check_submodule_status(submodule)

    def check_submodule_status(self, submodule: SubmoduleInfo):
        working_dir = f"/data/openpilot/{submodule.name}"

        if not os.path.exists(working_dir):
            submodule.status = "Not Initialized"
            submodule.color = WARNING
            return

        # Check changes
        success, output, _ = self.execute_git_command("git status --porcelain", working_dir)
        has_changes = success and bool(output.strip())

        # Check updates
        self.execute_git_command("git fetch", working_dir, timeout=30)
        success, output, _ = self.execute_git_command("git rev-list HEAD..@{u} --count", working_dir)
        has_updates = success and int(output.strip() or "0") > 0

        if has_changes:
            submodule.status = "Modified"
            submodule.color = WARNING
        elif has_updates:
            submodule.status = "Updates Available"
            submodule.color = PRIMARY
        else:
            submodule.status = "Up to Date"
            submodule.color = SUCCESS

        submodule.has_updates = has_updates
        submodule.has_changes = has_changes

        # Update button visibility
        if submodule.name in self.submodule_buttons:
            self.submodule_buttons[submodule.name]['update'].visible = has_updates
            self.submodule_buttons[submodule.name]['reset'].visible = has_changes

    def update_status(self):
        if self.is_onroad():
            self.updater_status = UpdaterStatus.ONROAD
        elif not self.check_internet():
            self.updater_status = UpdaterStatus.NO_INTERNET
        elif not self.check_ssh_valid():
            self.updater_status = UpdaterStatus.SSH_AUTH_FAILED
        else:
            success, _, _ = self.execute_git_command("git rev-parse --abbrev-ref --symbolic-full-name @{u}")
            if not success:
                self.updater_status = UpdaterStatus.NO_REMOTE_BRANCH
            else:
                self.updater_status = UpdaterStatus.OK

        # Update button states
        can_update = self.updater_status == UpdaterStatus.OK
        self.buttons['check_updates'].enabled = can_update
        self.buttons['update_repo'].enabled = can_update and self.git_status.has_updates
        self.buttons['update_all'].enabled = can_update
        self.buttons['repair_repo'].enabled = can_update

        # Update visibility
        self.buttons['update_repo'].visible = self.git_status.has_updates
        self.buttons['reset_repo'].visible = not self.git_status.clean

    def refresh_data(self):
        if self.command_dialog:
            return
        self.refresh_git_status()
        self.refresh_submodules()
        self.update_status()

    def check_for_updates(self):
        if self.command_dialog or self.is_onroad():
            return
        self.last_update_check = datetime.now()
        threading.Thread(target=self.refresh_data, daemon=True).start()

    def handle_button_click(self, button_name: str):
        if button_name == "check_updates":
            self.check_for_updates()
        elif button_name == "update_repo":
            cmd = "rm -f .git/index.lock && git fetch && git pull && git submodule update --init --recursive && scons -j$(nproc)"
            self.command_dialog = CommandDialog("Update Repository", cmd)
            self.command_dialog.start()
        elif button_name == "update_all":
            cmd = "rm -f .git/index.lock && git fetch && git pull --ff-only && git submodule update --init --recursive && scons -j$(nproc)"
            self.command_dialog = CommandDialog("Update All", cmd)
            self.command_dialog.start()
        elif button_name == "repair_repo":
            cmd = "rm -f .git/index.lock && git reset --hard HEAD && git clean -fd && git submodule update --init --recursive && scons -j$(nproc)"
            self.command_dialog = CommandDialog("Repair Repository", cmd)
            self.command_dialog.start()
        elif button_name == "reset_repo":
            cmd = "git reset --hard HEAD && git clean -fd"
            self.command_dialog = CommandDialog("Reset Changes", cmd)
            self.command_dialog.start()
        elif button_name == "show_commits":
            cmd = "git log --oneline --graph -20"
            self.command_dialog = CommandDialog("Recent Commits", cmd)
            self.command_dialog.start()
        elif button_name == "change_branch":
            cmd = "git branch -a"
            self.command_dialog = CommandDialog("Available Branches", cmd)
            self.command_dialog.start()
        elif button_name == "close_panel" and self.from_spinner:
            return "close_to_spinner"
        return None

    def handle_submodule_click(self, submodule_name: str, action: str):
        if action == "update":
            cmd = f"git submodule update --init --recursive {submodule_name}"
            self.command_dialog = CommandDialog(f"Update {submodule_name}", cmd)
            self.command_dialog.start()
        elif action == "reset":
            cmd = f"cd {submodule_name} && git reset --hard HEAD && git clean -fd"
            self.command_dialog = CommandDialog(f"Reset {submodule_name}", cmd)
            self.command_dialog.start()
        elif action == "repair":
            cmd = f"git submodule deinit -f {submodule_name} && rm -rf .git/modules/{submodule_name} && git submodule update --init --recursive {submodule_name}"
            self.command_dialog = CommandDialog(f"Repair {submodule_name}", cmd)
            self.command_dialog.start()

    def draw_status_indicator(self, x: float, y: float):
        if self.updater_status != UpdaterStatus.OK:
            # Status card
            status_card = Card(x, y, 300, 60)
            status_card.draw_background()

            # Icon
            rl.draw_circle(x + 30, y + 30, 8, WARNING)

            # Text
            rl.draw_text_ex(gui_app.font(), self.updater_status.value,
                           rl.Vector2(x + 50, y + 20), TEXT_SIZE, 0.0, TEXT_PRIMARY)

    def draw_main_section(self, y: float) -> float:
        # Main repository card
        card_height = 280
        card = Card(MARGIN, y, gui_app.width - 2 * MARGIN, card_height)
        card.draw_background()

        content_x = card.rect.x + CARD_PADDING
        content_y = card.rect.y + CARD_PADDING

        # Title
        rl.draw_text_ex(gui_app.font(), "Openpilot Repository",
                       rl.Vector2(content_x, content_y), TITLE_SIZE, 0.0, TEXT_PRIMARY)

        # Status indicator
        if self.updater_status != UpdaterStatus.OK:
            status_size = measure_text_cached(gui_app.font(), self.updater_status.value, TEXT_SIZE)
            status_x = card.rect.x + card.rect.width - CARD_PADDING - status_size.x - 30
            rl.draw_circle(int(status_x), int(content_y + 20), 6, rl.Color(*WARNING))
            rl.draw_text_ex(gui_app.font(), self.updater_status.value,
                           rl.Vector2(status_x + 15, content_y + 8), TEXT_SIZE, 0.0, WARNING)

        content_y += 50

        # Repository info
        repo_text = f"Repository: {self.current_repo}"
        rl.draw_text_ex(gui_app.font(), repo_text,
                       rl.Vector2(content_x, content_y), TEXT_SIZE, 0.0, TEXT_SECONDARY)

        content_y += 30
        branch_text = f"Branch: {self.current_branch}"
        rl.draw_text_ex(gui_app.font(), branch_text,
                       rl.Vector2(content_x, content_y), TEXT_SIZE, 0.0, TEXT_SECONDARY)

        # Status with color
        branch_size = measure_text_cached(gui_app.font(), branch_text, TEXT_SIZE)
        status_text = f"  •  {self.git_status.status_text}"
        rl.draw_text_ex(gui_app.font(), status_text,
                       rl.Vector2(content_x + branch_size.x, content_y), TEXT_SIZE, 0.0, self.git_status.status_color)

        content_y += 40

        # Commit info
        if self.git_status.commit_hash:
            commit_text = f"Latest: {self.git_status.commit_hash} - {self.git_status.commit_message}"
            wrapped = wrap_text(commit_text, SMALL_SIZE, card.rect.width - 2 * CARD_PADDING)
            for line in wrapped:
                rl.draw_text_ex(gui_app.font(), line,
                               rl.Vector2(content_x, content_y), SMALL_SIZE, 0.0, TEXT_MUTED)
                content_y += 25

        # Buttons
        button_y = card.rect.y + card_height - BUTTON_HEIGHT - 20
        button_x = content_x

        button_order = ['check_updates', 'update_repo', 'update_all', 'repair_repo', 'reset_repo', 'show_commits', 'change_branch']
        if self.from_spinner:
            button_order.append('close_panel')

        for btn_name in button_order:
            button = self.buttons[btn_name]
            if button.visible:
                # Auto-size button
                text_size = measure_text_cached(gui_app.font(), button.text, TEXT_SIZE)
                button.rect.width = text_size.x + 40
                button.rect.x = button_x
                button.rect.y = button_y
                button.draw()
                button_x += button.rect.width + BUTTON_SPACING

        # Last check time
        if self.last_update_check:
            time_str = self.last_update_check.strftime("Last checked: %m/%d %I:%M %p")
            rl.draw_text_ex(gui_app.font(), time_str,
                           rl.Vector2(content_x, button_y + BUTTON_HEIGHT + 10),
                           SMALL_SIZE, 0.0, TEXT_MUTED)

        return y + card_height + CARD_SPACING

    def draw_submodules_section(self, y: float) -> float:
        if not self.submodules:
            return y

        # Submodules card
        item_height = 60
        card_height = 60 + len(self.submodules) * item_height
        card = Card(MARGIN, y, gui_app.width - 2 * MARGIN, card_height)
        card.draw_background()

        content_x = card.rect.x + CARD_PADDING
        content_y = card.rect.y + CARD_PADDING

        # Title
        rl.draw_text_ex(gui_app.font(), "Submodules",
                       rl.Vector2(content_x, content_y), TITLE_SIZE, 0.0, TEXT_PRIMARY)
        content_y += 50

        # Submodules
        for submodule in self.submodules:
            # Name and status
            rl.draw_text_ex(gui_app.font(), submodule.name,
                           rl.Vector2(content_x, content_y), TEXT_SIZE, 0.0, TEXT_PRIMARY)

            name_size = measure_text_cached(gui_app.font(), submodule.name, TEXT_SIZE)
            rl.draw_text_ex(gui_app.font(), f"  •  {submodule.status}",
                           rl.Vector2(content_x + name_size.x, content_y), TEXT_SIZE, 0.0, submodule.color)

            # Buttons
            if submodule.name in self.submodule_buttons:
                button_x = card.rect.x + card.rect.width - 300
                for i, (action, button) in enumerate(self.submodule_buttons[submodule.name].items()):
                    if button.visible:
                        button.rect.x = button_x + i * 90
                        button.rect.y = content_y - 10
                        button.rect.width = 80
                        button.rect.height = 40
                        button.draw(SMALL_SIZE)

            content_y += item_height

        return y + card_height + CARD_SPACING

    def _render(self, rect: rl.Rectangle):
        # Update timers
        self.auto_update_timer.update()
        self.refresh_timer.update()
        self.param_timer.update()

        # Handle input
        mouse_pos = rl.get_mouse_position()
        mouse_pressed = rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT)
        mouse_released = rl.is_mouse_button_released(rl.MOUSE_BUTTON_LEFT)
        wheel_move = rl.get_mouse_wheel_move()

        # Command dialog priority
        if self.command_dialog:
            close_requested = self.command_dialog.update(mouse_pos, mouse_pressed, mouse_released, wheel_move)
            self.command_dialog.draw()
            if close_requested:
                if self.command_dialog and not self.command_dialog.running:
                    self.refresh_data()  # Refresh after successful commands
                self.command_dialog = None
            return

        # Update buttons
        for button in self.buttons.values():
            button.update(mouse_pos, mouse_pressed, mouse_released)

        for submodule_buttons in self.submodule_buttons.values():
            for button in submodule_buttons.values():
                button.update(mouse_pos, mouse_pressed, mouse_released)

        # Handle clicks
        for name, button in self.buttons.items():
            if button.clicked:
                result = self.handle_button_click(name)
                if result == "close_to_spinner":
                    return "close_to_spinner"

        for submodule_name, submodule_buttons in self.submodule_buttons.items():
            for action, button in submodule_buttons.items():
                if button.clicked:
                    self.handle_submodule_click(submodule_name, action)

        # Draw UI
        rl.draw_rectangle(0, 0, gui_app.width, gui_app.height, BG_COLOR)

        y = MARGIN
        y = self.draw_main_section(y)
        y = self.draw_submodules_section(y)


def main():
    gui_app.init_window("BP Updater")
    updater_panel = BPUpdaterPanel()

    for _ in gui_app.render():
        result = updater_panel.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
        if result == "close_to_spinner":
            break


if __name__ == "__main__":
    main()
