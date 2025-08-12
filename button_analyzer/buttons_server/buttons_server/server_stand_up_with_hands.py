#!/usr/bin/env python3

'''
АННОТАЦИЯ
Flask-сервер для управления состоянием кнопок в ROS 2-системе c помощью веб-интерфейса.
Предоставляет:
- Веб-интерфейс с отображением 5 кнопок (сброс, 1/7. Принять/Отдать, 2/6. Взять/Опустить,
  3/5. Держать низко, 4. Держать высоко)
- REST API для активации/деактивации кнопок
- Потокобезопасное управление состоянием через threading.Lock
- Логирование изменений с историей последних 100 событий
Требует HTML-шаблона из пакета buttons_server. Работает на порту 5000.

ANNOTATION
Flask server for managing button states in ROS 2 system with a web interface.
Provides:
- Web interface with 5 buttons (reset, 1/7.Accept/Give, 2/6.Take/Lower,
 3/5.Hold low, 4.Hold high)
- REST API for button activation/deactivation
- Thread-safe state management via threading.Lock
- Change logging with 100-entry history
Requires HTML template from buttons_server package. Runs on port 5000.
'''

import logging
import os
import threading
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, render_template_string, request


# ==============================================================================
# Configuration and Initialization
# ==============================================================================

app = Flask(__name__)

HOST = '0.0.0.0'

# Button states (extended list matching all buttons in HTML)
buttons = {
    "wave":          False,
    "photo":         False,
    "attention":     False,
    "selfie":        False,
    "1_offer_hand":  False,
    "2_shake_hand":  False,
    "1_offer_docs":  False,
    "2_grip_docs":   False,
    "3_hold_docs":   False,
    "4_give_docs":   False,
    "5_release_docs": False,
}

active_button = None
state_history = []
lock = threading.Lock()

# Load HTML template
package_share_dir = get_package_share_directory('buttons_server')
file_path = os.path.join(package_share_dir, 'html_skeleton_with_hands.html')
with open(file_path, 'r') as f:
    html_skeleton = f.read()


# ==============================================================================
# Web Interface Endpoints
# ==============================================================================

@app.route('/')
def home():
    """Render main page with button control interface."""
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
    Handle button activation/deactivation through API.
    Supports:
    - Activating a button (deactivates previous active button)
    - Deactivating current button
    - Resetting all buttons (btn=none)
    """
    global active_button, buttons
    btn = request.args.get('btn')
    active = request.args.get('active', 'true').lower() == 'true'

    # Handle reset request
    if btn == 'none':
        with lock:
            if active_button:
                buttons[active_button] = False
                log_change(f"'{active_button}' mode deactivated (reset)")

            active_button = None
            message = "All modes deactivated (reset)"
            log_change(message)

            return jsonify({
                "active": False,
                "active_button": None,
                "message": message,
            })

    # Validate button exists
    if btn not in buttons:
        return jsonify({"error": "Unknown button"}), 400

    with lock:
        # Handle deactivation of current button
        if not active and active_button == btn:
            buttons[btn] = False
            active_button = None
            message = get_button_message(btn, False)

        # Handle new activation
        elif active:
            # Deactivate previous button if exists
            if active_button:
                buttons[active_button] = False
                log_change(get_button_message(active_button, False))

            # Activate new button
            buttons[btn] = True
            active_button = btn
            message = get_button_message(btn, True)

        else:
            return jsonify({"error": "Invalid request"}), 400

        log_change(message)
        return jsonify({
            "active": buttons[btn],
            "active_button": active_button,
            "message": message,
        })


@app.route('/api/get_state')
def get_state():
    """API endpoint to get current button states and last change."""
    with lock:
        return jsonify({
            "active_button": active_button,
            "buttons": buttons,
            "last_change": state_history[-1] if state_history else "No changes",
        })


@app.route('/api/status')
def api_status():
    """API endpoint for server status and current button states."""
    return jsonify({
        'active_button': active_button,
        'buttons': buttons,
        'timestamp': datetime.now().isoformat(),
    })


# ==============================================================================
# Helper Functions
# ==============================================================================

def get_button_message(btn, active):
    """Return human-readable status message for button state changes."""
    messages = {
        "wave": {
            True:  "Activated 'Wave hand' mode",
            False: "Deactivated 'Wave hand' mode",
        },
        "photo": {
            True:  "Activated 'Take photo' mode",
            False: "Deactivated 'Take photo' mode",
        },
        "attention": {
            True:  "Activated 'Attention please' mode",
            False: "Deactivated 'Attention please' mode",
        },
        "selfie": {
            True:  "Activated 'Selfie' mode",
            False: "Deactivated 'Selfie' mode",
        },
        "1_offer_hand": {
            True:  "Activated 'Offer hand' mode",
            False: "Deactivated 'Offer hand' mode",
        },
        "2_shake_hand": {
            True:  "Activated 'Shake hand' mode",
            False: "Deactivated 'Shake hand' mode",
        },
        "1_offer_docs": {
            True:  "Activated 'Accept documents' mode",
            False: "Deactivated 'Accept documents' mode",
        },
        "2_grip_docs": {
            True:  "Activated 'Take documents' mode",
            False: "Deactivated 'Take documents' mode",
        },
        "3_hold_docs": {
            True:  "Activated 'Hold documents' mode",
            False: "Deactivated 'Hold documents' mode",
        },
        "4_give_docs": {
            True:  "Activated 'Give documents' mode",
            False: "Deactivated 'Give documents' mode",
        },
        "5_release_docs": {
            True:  "Activated 'Release documents' mode",
            False: "Deactivated 'Release documents' mode",
        },
    }
    return messages[btn][active]


def log_change(message):
    """Log state changes with timestamp, keeping history limited to 100 entries."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    state_history.append(log_entry)
    
    if len(state_history) > 100:
        state_history.pop(0)


# ==============================================================================
# Main Execution
# ==============================================================================

def main():
    """Start the Flask server."""
    # Reduce Flask logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    print(' * Running on http://' + HOST + ':5000/')
    
    app.run(host=HOST, port=5000, debug=False)
    


if __name__ == '__main__':
    main()