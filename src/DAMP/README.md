RC transmitter settings:

  * SWA -> Channel 5.
    Controls direction of the main motor. State *1* - forward, state *2* - back.
  * SWD -> Channel 6.
    Controls break. State *1* - off, state *2* - on.

Pinout:

  * Main outputs:

| Pin number\* | 1              | 2               | 3       | 4   | 5   | 6   | 7   | 8   |
|-|-|-|-|-|-|-|-|-|
| Pin function | Main motor pwm | Steer motor pwm | DNC\*\* | DNC | DNC | DNC | DNC | DNC |

  * AUX outputs:

| Pin number\* | 1                   | 2                    | 3                         | 4       | 5   | 6   |
|-|-|-|-|-|-|-|
 Pin function | Steer motor CW GPIO | Steer motor CCW GPIO | Main motor direction GPIO | DNC\*\* | DNC | DNC |

\* Pins numbers are specified according to the numeration here: https://pixhawk.org/modules/pixhawk

\*\* DNC - Do Not Connect