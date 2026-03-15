#!/usr/bin/env python3
"""
Z1 Keyboard Safety Node
=======================
Controllo da tastiera per la safety del braccio Z1.

Tasti:
  H   → HOME      — interrompe il task corrente e porta il braccio in home
  ESC → EMERGENCY — ferma tutto, porta in home, blocca la FSM in EMERGENCY
  R   → RESET     — da EMERGENCY: torna in HOMING → WAITING (riprende operatività)
  Q   → QUIT      — termina questo nodo (non la FSM)

Pubblica:  /z1_keyboard_cmd  (std_msgs/String)
Legge:     /z1_fsm/state     (std_msgs/String)  per visualizzazione stato

Avvio raccomandato in un terminale dedicato:
    ros2 run z1_vision z1_keyboard_safety

(Se lanciato senza terminale interattivo, il keyboard input è disabilitato
ma il nodo resta attivo per ricevere comandi via topic se necessario.)
"""

import sys
import os
import tty
import termios
import select
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Z1KeyboardSafety(Node):

    # Istruzioni mostrate a schermo
    _HEADER = (
        "\r\n"
        "╔══════════════════════════════════════════════╗\r\n"
        "║        Z1 Keyboard Safety Controller         ║\r\n"
        "╠══════════════════════════════════════════════╣\r\n"
        "║  H   = Home      (interrompi → vai in home) ║\r\n"
        "║  ESC = Emergency (ferma tutto → EMERGENCY)  ║\r\n"
        "║  R   = Reset     (da EMERGENCY → riprendi)  ║\r\n"
        "║  Q   = Quit      (chiudi questo nodo)        ║\r\n"
        "╚══════════════════════════════════════════════╝\r\n"
        "\r\n"
    )

    def __init__(self):
        super().__init__("z1_keyboard_safety")

        # ── Parametri ────────────────────────────────────────────────────
        self.declare_parameter("keyboard_cmd_topic", "/z1_keyboard_cmd")
        self.declare_parameter("fsm_state_topic",    "/z1_fsm/state")

        cmd_topic   = self.get_parameter("keyboard_cmd_topic").value
        state_topic = self.get_parameter("fsm_state_topic").value

        # ── Publisher / Subscriber ────────────────────────────────────────
        self._pub_cmd  = self.create_publisher(String, cmd_topic, 10)
        self._fsm_state = "UNKNOWN"
        self.create_subscription(String, state_topic, self._on_fsm_state, 10)

        self.get_logger().info(
            f"⌨️  z1_keyboard_safety pronto | cmd→{cmd_topic} | state←{state_topic}"
        )

        # ── Keyboard thread ───────────────────────────────────────────────
        self._running   = True
        self._kb_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True, name="kb_safety"
        )
        self._kb_thread.start()

    # ================================================================= #
    #  ROS CALLBACKS                                                      #
    # ================================================================= #

    def _on_fsm_state(self, msg: String):
        self._fsm_state = msg.data

    # ================================================================= #
    #  KEYBOARD LOOP                                                      #
    # ================================================================= #

    def _send_cmd(self, cmd: str, label: str):
        """Pubblica il comando e stampa feedback."""
        self._pub_cmd.publish(String(data=cmd))
        self._tty_write(f"\r\n  → {label} (FSM: {self._fsm_state})\r\n")

    def _tty_write(self, text: str):
        """Scrive sul terminale (compatibile con raw mode)."""
        sys.stdout.write(text)
        sys.stdout.flush()

    def _refresh_status(self):
        """Aggiorna la riga di stato (sovrascrive la riga corrente)."""
        line = (
            f"\r  FSM: {self._fsm_state:<28s} | "
            "H=Home  ESC=Emergency  R=Reset  Q=Quit  "
        )
        sys.stdout.write(line)
        sys.stdout.flush()

    def _keyboard_loop(self):
        """Loop bloccante che legge la tastiera in raw mode."""

        # Se non siamo in un terminale interattivo, non ha senso procedere
        if not sys.stdin.isatty():
            self.get_logger().warn(
                "⚠️  stdin non è un terminale reale — keyboard input disabilitato.\n"
                "    Avvia il nodo in un terminale separato:\n"
                "        ros2 run z1_vision z1_keyboard_safety"
            )
            return

        fd           = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            self._tty_write(self._HEADER)
            self._refresh_status()

            while self._running and rclpy.ok():

                # Attesa bloccante con timeout per poter aggiornare lo status
                ready, _, _ = select.select([sys.stdin], [], [], 0.5)

                if not ready:
                    # Nessun tasto premuto: aggiorna la riga di stato
                    self._refresh_status()
                    continue

                ch = sys.stdin.read(1)
                if not ch:
                    continue

                # ── ESC o escape sequence (frecce, F-keys ecc.) ──────
                if ch == '\x1b':
                    # Leggi eventuali caratteri aggiuntivi della sequenza
                    extra, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if extra:
                        # È una sequenza (es. \x1b[A per freccia su) → ignora
                        sys.stdin.read(2)
                    else:
                        # ESC puro → EMERGENCY
                        self._send_cmd("emergency", "🚨 EMERGENCY inviato!")
                    continue

                key = ch.lower()

                if key == 'h':
                    self._send_cmd("home", "🏠 HOME inviato!")

                elif key == 'r':
                    self._send_cmd("reset", "🔄 RESET inviato!")

                elif key == 'q' or ch == '\x03':
                    # Q o Ctrl+C → esci dal loop
                    self._tty_write("\r\n  Quit — chiusura nodo keyboard safety.\r\n")
                    self._running = False
                    break

                # Aggiorna la riga di stato dopo ogni tasto
                self._refresh_status()

        except Exception as e:
            # Ripristina il terminale anche in caso di eccezione
            self.get_logger().error(f"Errore keyboard loop: {e}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            sys.stdout.write("\r\n[z1_keyboard_safety] Terminato.\r\n")
            sys.stdout.flush()

    # ================================================================= #
    #  CLEANUP                                                            #
    # ================================================================= #

    def destroy_node(self):
        self._running = False
        super().destroy_node()


# ======================================================================= #
def main(args=None):
    rclpy.init(args=args)
    node = Z1KeyboardSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
