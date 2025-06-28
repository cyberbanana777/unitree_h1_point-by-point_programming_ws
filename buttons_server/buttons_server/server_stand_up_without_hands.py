#!/usr/bin/env python3

'''
АННОТАЦИЯ
Скрипт предназначен для запуска веб-сервера для управления кнопками.
Запуск осуществляется на localhost на порту 5000.
Опрос кнопок осуществляется с помощью API.
Структура сайта описана в файле html_skeleton.html в папке resource этого пакета.
'''

'''
ANNOTATION
The script is designed to launch a web server to manage buttons.
It is launched on localhost on port 5000.
The buttons are polled using the API.
The site structure is described in the html_skeleton.html file 
in the resource folder of this package.
'''

import logging
import os
import threading
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, render_template_string, request


app = Flask(__name__)

# Состояния кнопок
buttons = {
    "selfie": False,
    "wave": False,
    "photo": False,
    "attention": False
}
active_button = None
state_history = []
lock = threading.Lock()


package_share_dir = get_package_share_directory('buttons_server')
file_path = os.path.join(package_share_dir, 'html_skeleton_without_hands.html')
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
    
    if btn not in buttons:
        return jsonify({"error": "Неизвестная кнопка"}), 400
    
    with lock:
        # Если нажата уже активная кнопка - деактивируем
        if active_button == btn:
            buttons[btn] = False
            active_button = None
            message = get_button_message(btn, False)
        else:
            # Деактивируем предыдущую активную кнопку
            if active_button:
                buttons[active_button] = False
            
            # Активируем новую
            buttons[btn] = True
            active_button = btn
            message = get_button_message(btn, True)
        
        log_change(message)
        
        return jsonify({
            "active": buttons[btn],
            "active_button": active_button,
            "message": message,
        })


def get_button_message(btn, active):
    """
    Возвращает сообщение для лога в зависимости от состояния кнопки.
    
    Args:
        btn (str): Идентификатор кнопки
        active (bool): Флаг активности кнопки
    
    Returns:
        str: Текстовое описание изменения состояния
    """
    messages = {
        "selfie": {
            True: "Активирован режим 'Сэлфи'",
            False: "Режим 'Сэлфи' деактивирован",
        },
        "wave": {
            True: "Активирован режим 'Помахать рукой'",
            False: "Режим 'Помахать рукой' деактивирован",
        },
        "photo": {
            True: "Активирован режим 'Приглашаю на фото'",
            False: "Режим 'Приглашаю на фото' деактивирован",
        },
        "attention": {
            True: "Свободная кнопка активирована",
            False: "Свободная кнопка деактивирована",
        },
    }
    return messages[btn][active]



@app.route('/api/get_state')
def get_state():
    """API-метод для получения текущего состояния кнопок."""
    with lock:
        return jsonify({
            "active_button": active_button,
            "buttons": buttons,
            "last_change": state_history[-1] if state_history else "Нет изменений",
        })


def log_change(message):
    """Логирование изменений состояния с ограничением истории до 100 записей."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_entry = f"[{timestamp}] {message}"
    state_history.append(log_entry)
    # print(log_entry) - Расскомментить для вывода логов
    if len(state_history) > 100:
        state_history.pop(0)


@app.route('/api/status')
def api_status():
    """API-метод для проверки статуса сервера."""
    return jsonify({
        'active_button': active_button,
        'buttons': buttons,
        'timestamp': datetime.now().isoformat(),
    })


def main():
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)

    app.run(host='0.0.0.0', port=5000, debug=False)


if __name__ == '__main__':
    main()