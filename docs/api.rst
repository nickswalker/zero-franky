API reference
=============

The public Python API is generated from the package docstrings.

Robot
-----

``zero_franky.Robot`` is an alias for ``zero_franky.zmq_client.RobotProxy``.

.. autoclass:: zero_franky.zmq_client.RobotProxy
   :members:

Connection setup
----------------

.. autofunction:: zero_franky.setup_zero_franky



Tracker proxies
---------------

Joint impedance
~~~~~~~~~~~~~~~

.. autoclass:: zero_franky.zmq_client.JointImpedanceTrackerProxy
   :members:
   :inherited-members:

Cartesian impedance
~~~~~~~~~~~~~~~~~~~

.. autoclass:: zero_franky.zmq_client.CartesianImpedanceTrackerProxy
   :members:
   :inherited-members:

Torque
~~~~~~

.. autoclass:: zero_franky.zmq_client.TorqueTrackerProxy
   :members:
   :inherited-members:

Value types mirror the `franky` API where applicable. Use the local stand-ins from
`franky Python API <https://timschneider42.github.io/franky/api/python.html>`_ for type semantics.
