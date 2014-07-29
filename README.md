##speech_test

A node that can play sound files, accept PAU messages (as defined in the pau2motors node), modify the jaw on them according to the currently playing sound volume and send them back out.

###Topics

Listens to `cmd_speak` for a string - a soundfile name in the **res** folder.
Listens to `speechtest_face_in` for PAU face messages
Publishes on `cmd_face_pau` modified PAU messages

The node usually is supposed to publish on `cmd_face_pau`, when using this node, is meant to be remapped to `speechtest_face_in`.

E.g. `rosrun basic_head_api head_ctrl.py cmd_face_pau:=speechtest_face_in`
