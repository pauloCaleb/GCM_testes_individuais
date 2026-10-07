"""Fumaça da GUI em modo offscreen, usando o simulador."""

import time

import pytest

pytest.importorskip("PySide6")
pytest.importorskip("pyqtgraph")

from PySide6 import QtWidgets  # noqa: E402

from gcm_test.gui import MainWindow  # noqa: E402


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def pump(app, sec):
    end = time.time() + sec
    while time.time() < end:
        app.processEvents()
        time.sleep(0.01)


def test_gui_flow(app):
    w = MainWindow()
    w.show()
    w.connect_to("DEMO")
    pump(app, 1.5)
    assert w.ready and w.panels.isEnabled()

    w.led_btns[1].click()          # LED 2 estava ligado no simulador -> desliga
    pump(app, 0.4)
    assert not w.led_btns[1].isChecked()

    w.en_btn.click()
    w.sliders[0].setValue(20)
    pump(app, 1.0)
    assert w.en_btn.isChecked()
    assert w.act_lbls[0].text().startswith("aplicado: +20")

    w._do_stop()
    pump(app, 0.4)
    assert not w.en_btn.isChecked()
    assert w.sliders[0].value() == 0

    w.disconnect_from()
    assert not w.client.connected
    w.close()


def test_gui_watchdog_banner(app):
    w = MainWindow()
    w.show()
    w.connect_to("DEMO")
    pump(app, 1.0)
    w.wd_spin.setValue(100)
    w._apply_cfg()
    w.en_btn.click()
    w.sliders[0].setValue(5)
    pump(app, 0.2)
    w.hb_timer.stop()              # sem heartbeat -> o firmware (simulado) dispara o watchdog
    pump(app, 0.8)
    assert w.banner.isVisible()
    assert not w.en_btn.isChecked()
    w.disconnect_from()
    w.close()
