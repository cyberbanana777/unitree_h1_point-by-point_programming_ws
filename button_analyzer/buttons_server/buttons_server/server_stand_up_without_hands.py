#!/usr/bin/env python3

'''
АННОТАЦИЯ
Flask-сервер для управления состоянием кнопок в ROS 2-системе. Обеспечивает:
- Веб-интерфейс с отображением 4 кнопок (selfie, wave, photo, attention)
- REST API для переключения состояний кнопок (эксклюзивная активация)
- Потокобезопасную работу через threading.Lock
- Логирование изменений с историей последних 100 событий
Использует HTML-шаблон из пакета buttons_server. Запускается на порту 5000.

ANNOTATION
Flask server for managing button states in ROS 2 system. Provides:
- Web interface displaying 4 buttons (selfie, wave, photo, attention)
- REST API for toggling button states (exclusive activation)
- Thread-safe operation via threading.Lock
- Change logging with 100-entry history
Uses HTML template from buttons_server package. Runs on port 5000.
'''


import logging
import os
import threading
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, render_template_string, request


# ==============================================================================
# Server Configuration and Initialization
# ==============================================================================

app = Flask(__name__)

HOST = '0.0.0.0'

# Button states with initial values
buttons = {
    "selfie":    False,
    "wave":      False,
    "photo":     False,
    "attention": False,
}

active_button = None          # Currently active button
state_history = []            # History of state changes
lock = threading.Lock()       # Thread synchronization lock

# Load HTML template
package_share_dir = get_package_share_directory('buttons_server')
file_path = os.path.join(package_share_dir, 'html_skeleton_without_hands.html')

with open(file_path, 'r') as f:
    html_skeleton = f.read()


# ==============================================================================
# Web Interface Endpoints
# ==============================================================================

@app.route('/')
def home():
    """Render the main control interface page."""
    return render_template_string(
        html_skeleton,
        buttons=buttons,
        active_button=active_button,
        state_history=state_history,
    )


# ==============================================================================
# API Endpoints
# ==============================================================================

@app.route('/activate')
def activate():
    """
    Handle button activation/deactivation via API.
    
    Returns:
        JSON response with new state and status message
    """
    global active_button, buttons
    btn = request.args.get('btn')

    if btn not in buttons:
        return jsonify({"error": "Unknown button"}), 400

    with lock:
        # Deactivate if clicking the active button
        if active_button == btn:
            buttons[btn] = False
            active_button = None
            message = get_button_message(btn, False)
        
        # Activate new button
        else:
            if active_button:
                buttons[active_button] = False

            buttons[btn] = True
            active_button = btn
            message = get_button_message(btn, True)

        log_change(message)

        return jsonify({
            "active": buttons[btn],
            "active_button": active_button,
            "message": message,
        })


@app.route('/api/get_state')
def get_state():
    """Get current button states and last change information."""
    with lock:
        return jsonify({
            "active_button": active_button,
            "buttons": buttons,
            "last_change": state_history[-1] if state_history else "No changes",
        })


@app.route('/api/status')
def api_status():
    """Get server status and current button states with timestamp."""
    return jsonify({
        'active_button': active_button,
        'buttons': buttons,
        'timestamp': datetime.now().isoformat(),
    })


# ==============================================================================
# Helper Functions
# ==============================================================================

def get_button_message(btn, active):
    """
    Generate human-readable status message for button state changes.
    
    Args:
        btn: Button identifier (str)
        active: New button state (bool)
        
    Returns:
        Appropriate status message (str)
    """
    messages = {
        "selfie": {
            True:  "Activated 'Selfie' mode",
            False: "Deactivated 'Selfie' mode",
        },
        "wave": {
            True:  "Activated 'Wave hand' mode",
            False: "Deactivated 'Wave hand' mode",
        },
        "photo": {
            True:  "Activated 'Take photo' mode",
            False: "Deactivated 'Take photo' mode",
        },
        "attention": {
            True:  "Activated custom button",
            False: "Deactivated custom button",
        },
    }
    return messages[btn][active]


def log_change(message):
    """
    Log state changes with timestamp and maintain history.
    
    Keeps only the most recent 100 entries in history.
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    state_history.append(log_entry)
    
    if len(state_history) > 100:
        state_history.pop(0)


# ==============================================================================
# Main Execution
# ==============================================================================

def main():
    """Configure and start the Flask server."""
    # Reduce Flask logging verbosity
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    print(' * Running on http://' + HOST + ':5000/')

    app.run(host=HOST, port=5000, debug=False)


if __name__ == '__main__':
    main()