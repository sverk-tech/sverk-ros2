# Русская озвучка RadioMaster Pocket

RadioMaster Pocket — пульт управления Обриком, работающий по протоколу ELRS (ExpressLRS). По умолчанию системные сообщения воспроизводятся на английском. Инструкция описывает, как загрузить готовые профили и переключить озвучку на русский язык.

---

## Шаг 1. Скачайте архив с настройками

Скачайте [архив с готовыми настройками](https://www.google.com/url?q=https://drive.google.com/drive/folders/13V-xxbfMj9bZtWU3VrE0vCBnuwDvGQ82?usp%3Ddrive_link&sa=D&source=docs&ust=1773057066587064&usg=AOvVaw2IXGQhfwC7kRUnIrMfr3Ho). Внутри три файла:

| Файл | Что содержит |
|------|-------------|
| model00.yml | Профиль настроек для управления Обриком |
| radio.yml | Настройки самого передатчика (пульта) |
| ru | Папка с файлами русской озвучки |

---

## Шаг 2. Подключите пульт к компьютеру

Подключите пульт к компьютеру через верхний разъём USB-C (он находится на верхней грани пульта).

<img class="center" src="../assets/1sverk/hardware_control/DSCF0843.png" alt="usb-разъём">

На экране пульта появится меню. Выберите **USB Storage (SD)**.

<img class="center" src="../assets/1sverk/hardware_control/DSCF0783.png" alt="экран выбора usb">

Пульт появится на компьютере как флешка.

---

## Шаг 3. Скопируйте файлы

Откройте хранилище пульта на компьютере.

В папку **MODELS** скопируйте файл `model00.yml`.

> **Примечание** Если в папке MODELS уже есть файл с таким именем, удалите его или подтвердите замену при копировании.

<img class="center" src="../assets/1sverk/hardware_control/screen_monitor1.png" alt="папки1">

В папку **RADIO** скопируйте файл `radio.yml`.

> **Примечание** Если в папке RADIO уже есть файл с таким именем, удалите его или подтвердите замену при копировании.

<img class="center" src="../assets/1sverk/hardware_control/screen_monitor2.png" alt="папки2">

В папку **SOUNDS** скопируйте папку `ru` целиком.

<img class="center" src="../assets/1sverk/hardware_control/screen_monitor3.png" alt="папки3">

Отключите пульт от компьютера.

---

## Шаг 4. Активируйте русскую озвучку

На пульте нажмите кнопку **SYS**.

<img class="center" src="../assets/1sverk/hardware_control/DSCF0763-.png" alt="кнопка sys">

С помощью кнопки **PAGE>** перейдите в третью вкладку (3/7) **RADIO SETUP**.

<img class="center" src="../assets/1sverk/hardware_control/DSCF0763_1.png" alt="кнопка sys">

Пролистайте вниз до строки **voice language** и выберите **Russian**.

Готово. Пульт будет говорить по-русски.

---

## Что означает мигание на приёмнике

Приёмник ELRS стоит на дроне и принимает сигналы от пульта. По миганию светодиода на нём можно понять, что происходит:

<table class="type_table">
    <tr>
        <td><img class="center" src="../assets/1sverk/hardware_control/gif1.gif" width=100></td>
        <td>Непрерывный свет</td>
        <td>Приёмник подключён к пульту, или включён режим загрузчика</td>
    </tr>
    <tr>
        <td><img class="center" src="../assets/1sverk/hardware_control/gif2.gif" width=100></td>
        <td>Медленное мигание (500 мс вкл/выкл)</td>
        <td>Ожидание сигнала от пульта</td>
    </tr>
    <tr>
        <td><img class="center" src="../assets/1sverk/hardware_control/gif3.gif" width=100></td>
        <td>Быстрое мигание (25 мс вкл/выкл)</td>
        <td>Режим Wi-Fi включён (для обновления прошивки приёмника)</td>
    </tr>
</table>

Если приёмник медленно мигает, а пульт включён и рядом, значит они не связаны. Нужно выполнить привязку (binding).
