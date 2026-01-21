import urllib.request
import urllib.parse
import json
import time


APEX_API_KEY = "YOUR_API_KEY"

current_mode = "ranked"

def get_mode_title(mode="ranked"):
    return "Рейтинговый режим" if mode == "ranked" else "Публичные матчи"

def get_real_map_data(mode="ranked"):
    try:
        url = f"https://api.mozambiquehe.re/maprotation?version=2&auth={APEX_API_KEY}"
        response = urllib.request.urlopen(url, timeout=10)
        data = json.loads(response.read().decode('utf-8'))
        
        if data.get(mode) and data[mode].get("current"):
            target_data = data[mode]
            print(f"✅ Использую запрошенный режим: {mode}")
        else:
            print(f"⚠️ Режим {mode} недоступен, пробую ranked...")
            if data.get("ranked") and data["ranked"].get("current"):
                target_data = data["ranked"]
                mode = "ranked" 
            else:
                return None
        
        if not target_data or not target_data.get("current"):
            return None
            
        current_map = target_data["current"]
        next_map = target_data.get("next", {})
        
        return {
            "current": current_map["map"],
            "timer": current_map["remainingTimer"],
            "next": next_map.get("map", "Неизвестно")
        }
        
    except Exception as e:
        print(f"Ошибка: {e}")
        return None

def switch_mode(mode):
    global current_mode
    current_mode = mode
    print(f"Переключил режим на: {mode}")

BOT_TOKEN = "YOUR_API_KEY"
BASE_URL = f"https://api.telegram.org/bot{BOT_TOKEN}"

def make_request(url, data=None):
    try:
        if data:
            data_bytes = json.dumps(data).encode('utf-8')
            request = urllib.request.Request(
                url, 
                data=data_bytes,
                headers={'Content-Type': 'application/json'}
            )
        else:
            request = urllib.request.Request(url)
        
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.loads(response.read().decode('utf-8'))
    except Exception as e:
        print(f"Ошибка запроса: {e}")
        return {"ok": False}

def get_updates(offset=None):
    url = f"{BASE_URL}/getUpdates"
    params = {"timeout": 30}
    if offset:
        params["offset"] = offset
        
    url_with_params = f"{url}?{urllib.parse.urlencode(params)}"
    return make_request(url_with_params)

def send_message(chat_id, text):
    url = f"{BASE_URL}/sendMessage"
    data = {"chat_id": chat_id, "text": text}
    return make_request(url, data)

def handle_command(message):
    text = message.get("text", "")
    chat_id = message["chat"]["id"]
    user_name = message["from"].get("first_name", "друг")
    
    print(f"👤 {user_name}: {text}")
    
    if text.startswith("/start"):
        buttons = [
            [{"text": "Рейтинг", "callback_data": "mode_ranked"}],
            [{"text": "Паблики", "callback_data": "mode_battle_royale"}]
        ]

        send_message_with_buttons(
            chat_id,
            f"Привет, {user_name}! \nВыбери режим для просмотра карт:",
            buttons
        )

    elif text.startswith("/help"):
        send_message(chat_id, "Команды:\n/start - начало\n/help - помощь\n/map ranked - рейтинговый пул\n/map pubs - обычный пул")
    elif text.startswith("/map") or text.startswith("/карта"):
        if "паб" in text.lower() or "pubs" in text.lower():
            mode = "battle_royale"
            mode_name = "Публичные матчи"
        else:
            mode = "ranked"
            mode_name = "Рейтинговый режим"
        
        map_data = get_real_map_data(mode)
        
        if map_data:
            message_text = (
                f"{mode_name}\n\n"
                f"Текущая карта: {map_data['current']}\n"
                f"До смены: {map_data['timer']}\n"
                f"Следующая: {map_data['next']}"
            )
        else:
            message_text = "Не удалось получить данные карт"
        
        send_message(chat_id, message_text)
    else:
        send_message(chat_id, "Не понимаю. Используй /help")

def send_message_with_buttons(chat_id, text, buttons):
    url = f"{BASE_URL}/sendMessage"
    
    keyboard = []
    for row in buttons:
        keyboard_row = []
        for button in row:
            keyboard_row.append({
                "text": button["text"],
                "callback_data": button["callback_data"]
            })
        keyboard.append(keyboard_row)
    
    data = {
        "chat_id": chat_id,
        "text": text,
        "reply_markup": {
            "inline_keyboard": keyboard
        }
    }
    
    try:
        response = urllib.request.urlopen(
            url, 
            data=json.dumps(data).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
        return json.loads(response.read().decode('utf-8'))
    except Exception as e:
        print(f"Ошибка отправки с кнопками: {e}")
        return {"ok": False}

def send_message_with_buttons(chat_id, text, buttons):
    """Отправляем сообщение с инлайн-кнопками"""
    url = f"{BASE_URL}/sendMessage"
    
    keyboard = []
    for row in buttons:
        keyboard_row = []
        for button in row:
            keyboard_row.append({
                "text": button["text"],
                "callback_data": button["callback_data"]
            })
        keyboard.append(keyboard_row)
    
    data = {
        "chat_id": chat_id,
        "text": text,
        "reply_markup": {
            "inline_keyboard": keyboard
        }
    }
    
    try:
        data_bytes = json.dumps(data).encode('utf-8')
        request = urllib.request.Request(
            url, 
            data=data_bytes,
            headers={'Content-Type': 'application/json'}
        )
        response = urllib.request.urlopen(request, timeout=10)
        return json.loads(response.read().decode('utf-8'))
    except Exception as e:
        print(f"Ошибка отправки с кнопками: {e}")
        return {"ok": False}
    
def edit_message_with_buttons(chat_id, message_id, text, buttons):
    url = f"{BASE_URL}/editMessageText"
    
    keyboard = []
    for row in buttons:
        keyboard_row = []
        for button in row:
            keyboard_row.append({
                "text": button["text"],
                "callback_data": button["callback_data"]
            })
        keyboard.append(keyboard_row)
    
    data = {
        "chat_id": chat_id,
        "message_id": message_id,
        "text": text,
        "reply_markup": {
            "inline_keyboard": keyboard
        }
    }
    
    try:
        data_bytes = json.dumps(data).encode('utf-8')
        request = urllib.request.Request(
            url, 
            data=data_bytes,
            headers={'Content-Type': 'application/json'}
        )
        response = urllib.request.urlopen(request, timeout=10)
        return json.loads(response.read().decode('utf-8'))
    except Exception as e:
        print(f"Ошибка редактирования: {e}")
        return {"ok": False}

def handle_callback(update):
    callback_query = update["callback_query"]
    chat_id = callback_query["message"]["chat"]["id"]
    message_id = callback_query["message"]["message_id"]
    data = callback_query["data"]
    user_name = callback_query["from"].get("first_name", "друг")
    
    print(f"🔄 {user_name} нажал кнопку: {data}")
    
    if data.startswith("mode_"):
        mode = data.replace("mode_", "")
        map_data = get_real_map_data(mode)
        
        if map_data:
            mode_name = "Рейтинговый режим" if mode == "ranked" else "Публичные матчи"
            message_text = (
                f"🗺 {mode_name}\n\n"
                f"🎯 Текущая карта: {map_data['current']}\n"
                f"⏰ До смены: {map_data['timer']}\n"
                f"🔜 Следующая: {map_data['next']}"
            )
        else:
            message_text = "Не удалось получить данные карт"
        
        buttons = [
            [{"text": " Рейтинг", "callback_data": "mode_ranked"}],
            [{"text": " Паблики", "callback_data": "mode_battle_royale"}],
            [{"text": " Обновить", "callback_data": f"mode_{mode}"}]
        ]
        
        edit_message_with_buttons(chat_id, message_id, message_text, buttons)
    
    answer_url = f"{BASE_URL}/answerCallbackQuery"
    answer_data = {"callback_query_id": callback_query["id"]}
    
    try:
        urllib.request.urlopen(
            answer_url,
            data=json.dumps(answer_data).encode('utf-8'),
            headers={'Content-Type': 'application/json'}
        )
    except:
        pass

def main():
    print("🚀 Запускаю бота...")
    last_update_id = None
    
    try:
        while True:
            print("📡 Проверяю сообщения...", end=" ")
            updates = get_updates(last_update_id)
            
            if updates.get("ok") and updates["result"]:
                print(f"Найдено: {len(updates['result'])}")
                for update in updates["result"]:
                    last_update_id = update["update_id"] + 1
                    
                    if "message" in update:
                        handle_command(update["message"])
                    elif "callback_query" in update:
                        handle_callback(update)
            else:
                print("нет новых")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nБот остановлен")

if __name__ == "__main__":
    main()

