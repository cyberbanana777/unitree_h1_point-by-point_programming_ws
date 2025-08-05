import logging
import os
import threading
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, render_template_string, request

#!/usr/bin/env python3


app = Flask(__name__)

# Состояния кнопок (расширенный список для всех кнопок из HTML)
buttons = {
    "wave": False,
    "photo": False,
    "attention": False,
    "selfie": False,
    "1_offer_hand": False,
    "2_shake_hand": False,
    "1_offer_docs": False,
    "2_grip_docs": False,
    "3_hold_docs": False,
    "4_give_docs": False,
    "5_release_docs": False,
}
active_button = None
state_history = []
lock = threading.Lock()

package_share_dir = get_package_share_directory('buttons_server')
file_path = os.path.join(package_share_dir, 'html_skeleton_with_hands.html')
with open(file_path, 'r') as f:
    html_skeleton = f.read()


@app.route('/')
def home():
    """Главная страница с интерфейсом управления кнопками."""
    return render_template_string(
        html_skeleton,
        buttons=buttons,
        active_button=active_button,
        state_history=state_history,
    )


@app.route('/activate')
def activate():
    """Активация/деактивация кнопок через API."""
    global active_button, buttons
    btn = request.args.get('btn')
    active = request.args.get('active', 'true').lower() == 'true'

    # Обработка запроса сброса (когда btn=none)
    if btn == 'none':
        with lock:
            # Деактивируем текущую активную кнопку, если есть
            if active_button:
                buttons[active_button] = False
                log_change(f"Режим '{active_button}' деактивирован (сброс)")

            active_button = None
            message = "Все режимы деактивированы (сброс)"
            log_change(message)

            return jsonify(
                {
                    "active": False,
                    "active_button": None,
                    "message": message,
                }
            )

    # Проверка существования кнопки
    if btn not in buttons:
        return jsonify({"error": "Неизвестная кнопка"}), 400

    with lock:
        # Если деактивируем текущую активную кнопку
        if not active and active_button == btn:
            buttons[btn] = False
            active_button = None
            message = get_button_message(btn, False)
        elif active:
            # Деактивируем предыдущую активную кнопку
            if active_button:
                buttons[active_button] = False
                log_change(get_button_message(active_button, False))

            # Активируем новую
            buttons[btn] = True
            active_button = btn
            message = get_button_message(btn, True)
        else:
            return jsonify({"error": "Некорректный запрос"}), 400

        log_change(message)

        return jsonify(
            {
                "active": buttons[btn],
                "active_button": active_button,
                "message": message,
            }
        )


def get_button_message(btn, active):
    """
    Возвращает сообщение для лога в зависимости от состояния кнопки.
    """
    messages = {
        "wave": {
            True: "Активирован режим 'Помахать рукой'",
            False: "Режим 'Помахать рукой' деактивирован",
        },
        "photo": {
            True: "Активирован режим 'Пригласить на фото'",
            False: "Режим 'Пригласить на фото' деактивирован",
        },
        "attention": {
            True: "Активирован режим 'Обратите внимание'",
            False: "Режим 'Обратите внимание' деактивирован",
        },
        "selfie": {
            True: "Активирован режим 'Сэлфи'",
            False: "Режим 'Сэлфи' деактивирован",
        },
        "1_offer_hand": {
            True: "Режим 'Подать руку' активирован",
            False: "Режим 'Подать руку' деактивирован",
        },
        "2_shake_hand": {
            True: "Режим 'Пожать руку' активирован",
            False: "Режим 'Пожать руку' деактивирован",
        },
        "1_offer_docs": {
            True: "Режим 'Принять документы' активирован",
            False: "Режим 'Принять документы' деактивирован",
        },
        "2_grip_docs": {
            True: "Режим 'Взять документы' активирован",
            False: "Режим 'Взять документы' деактивирован",
        },
        "3_hold_docs": {
            True: "Режим 'Держать документы' активирован",
            False: "Режим 'Держать документы' деактивирован",
        },
        "4_give_docs": {
            True: "Режим 'Отдать документы' активирован",
            False: "Режим 'Отдать документы' деактивирован",
        },
        "5_release_docs": {
            True: "Режим 'Отпустить документы' активирован",
            False: "Режим 'Отпустить документы' деактивирован",
        },
    }
    return messages[btn][active]


@app.route('/api/get_state')
def get_state():
    """API-метод для получения текущего состояния кнопок."""
    with lock:
        return jsonify(
            {
                "active_button": active_button,
                "buttons": buttons,
                "last_change": (
                    state_history[-1] if state_history else "Нет изменений"
                ),
            }
        )


def log_change(message):
    """Логирование изменений состояния с ограничением истории до 100 записей."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    state_history.append(log_entry)
    if len(state_history) > 100:
        state_history.pop(0)


@app.route('/api/status')
def api_status():
    """API-метод для проверки статуса сервера."""
    return jsonify(
        {
            'active_button': active_button,
            'buttons': buttons,
            'timestamp': datetime.now().isoformat(),
        }
    )


def main():
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000, debug=False)


if __name__ == '__main__':
    main()
