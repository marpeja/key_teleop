import curses

# For 'q' keystroke exit
import os
import signal
import time

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header
from std_msgs.msg import Int8


#TextWindow crea una ventana en la que se muestran los controles que se pueden realizar, y captura las teclas pulsadas mientras se
#está en dicha pantalla. Divide toda la pantalla en el número de lineas que se asigne, en este caso 10.
class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines
    
    #read_key se encarga de recoger el código de la tecla pulsada
    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()
        
    #Método para escribir por pantalla el texto que se pase como argumento, en la línea también pasada como argumento
    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            # TODO(artivis) Why are those floats ??
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

#Clase principal donde se encuentra toda la lógica. La interfaz de la pantalla se pasa como argumento, de manera que se pueda acceder
#a la tecla pulsada y al texto que se está mostrando en la misma
class SimpleKeyTeleop(Node):

    def __init__(self, interface):
        super().__init__('key_teleop')
        
        #Interface aloja a la pantalla creada
        self._interface = interface

        #Creamos todos los publishers que enviarán un valor según la tecla pulsada a la lógica del programa
        self._publish_w = self.create_publisher(Int8, 'value_w', 10)
        self._publish_a = self.create_publisher(Int8, 'value_a', 10)
        self._publish_s = self.create_publisher(Int8, 'value_s', 10)
        self._publish_d = self.create_publisher(Int8, 'value_d', 10)
        self._hz = self.declare_parameter('hz', 5).value

        self._last_pressed = {}                            
        self._clear_msg = Int8()
        self._clear_msg.data = 0
        self._clear = True
        self._w = 0
        self._a = 0
        self._s = 0
        self._d = 0

    #Movement bindings simplemente asigna un valor al código de la tecla pulsada, en este caso de momento todos a 1
    movement_bindings = {
        ord('w'):1,
        ord('a'):1,
        ord('s'):1,
        ord('d'):1,
    }

    def run(self):
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                self.get_logger().info('Publishing: {}'.format(keycode))
                # if keycode is None:
                #     # if self._w == 1:
                #     #     #self._w = 0
                #     #     #self._publish_w.publish(self._clear_msg)
                #     # elif self._a == 1:
                #     #     self._a = 0
                #     #     self._publish_a.publish(self._clear_msg)
                #     # elif self._s == 1:
                #     #     self._s = 0
                #     #     self._publish_s.publish(self._clear_msg)
                #     # elif self._d == 1:
                #     #     self._d = 0
                #     #     self._publish_d.publish(self._clear_msg)
                   
                #     break
                if keycode is None: 
                    if self._clear is True:
                        self._clearAll()
                    break
                #self._key_pressed(keycode)
                #self._set_velocity()
                self._sendVelocity(keycode)
                self._publish()
            # TODO(artivis) use Rate once available
                time.sleep(1.0/self._hz)

    def _set_velocity(self):
        now = self.get_clock().now()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < Duration(seconds=0.2):
                keys.append(a)
        for k in keys:
            if k == ord('w'):
                #if self._w != self.movement_bindings[k]:
                self._w = self.movement_bindings[k]
                msg = Int8()
                msg.data = self._w
                self._publish_w.publish(msg)
            elif k == ord('a'):
                if self._a != self.movement_bindings[k]:
                    self._a = self.movement_bindings[k]
                    msg = Int8()
                    msg.data = self._a
                    self._publish_a.publish(msg)
            elif k == ord('s'):
                if self._s != self.movement_bindings[k]:
                    self._s = self.movement_bindings[k]
                    msg = Int8()
                    msg.data = self._s
                    self._publish_s.publish(msg)
            elif k == ord('d'):
                if self._d != self.movement_bindings[k]:
                    self._d = self.movement_bindings[k]
                    msg = Int8()
                    msg.data = self._d
                    self._publish_d.publish(msg)

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            # TODO(artivis) no rclpy.signal_shutdown ?
            os.kill(os.getpid(), signal.SIGINT)
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = self.get_clock().now()
            
    def _sendVelocity(self, keycode):
        if keycode == ord('q'):
            self._running = False
            # TODO(artivis) no rclpy.signal_shutdown ?
            os.kill(os.getpid(), signal.SIGINT)
        elif keycode in self.movement_bindings:
            self._clear = True
            if keycode == ord('w'):
                self._w = self.movement_bindings[keycode]
                msg = Int8()
                msg.data = self._w
                self._publish_w.publish(msg)
            elif keycode == ord('a'):
                self._a = self.movement_bindings[keycode]
                msg = Int8()
                msg.data = self._a
                self._publish_a.publish(msg)
            elif keycode == ord('s'):
                self._s = self.movement_bindings[keycode]
                msg = Int8()
                msg.data = self._s
                self._publish_s.publish(msg)
            elif keycode == ord('d'):
                self._d = self.movement_bindings[keycode]
                msg = Int8()
                msg.data = self._d
                self._publish_d.publish(msg)
                    
    def _clearAll(self):
        self._clear = False
        self._publish_w.publish(self._clear_msg)
        self._publish_a.publish(self._clear_msg)
        self._publish_s.publish(self._clear_msg)
        self._publish_d.publish(self._clear_msg)
        self._w = 0
        self._a = 0
        self._s = 0
        self._d = 0
        

    def _publish(self):
        self._interface.clear()
        # self._interface.write_line(2, 'Linear: %f, Angular: %f' % (self._linear, self._angular))
        # self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
        self._interface.write_line(2, 'w: %f' % (self._w))
        self._interface.write_line(3, 'a: %f' % (self._a))
        self._interface.write_line(4, 's: %f' % (self._s))
        self._interface.write_line(5, 'd: %f' % (self._d))
        self._interface.write_line(8, 'Use arrow keys to move, q to exit.')
        self._interface.refresh()

        # if self._publish_stamped_twist:
        #     twist = self._make_twist_stamped(self._linear, self._angular)
        # else:
        #     twist = self._make_twist(self._linear, self._angular)

        #self._pub_cmd.publish(twist)


def execute(stdscr):
    rclpy.init()

    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

    app.destroy_node()
    rclpy.shutdown()


def main():
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()