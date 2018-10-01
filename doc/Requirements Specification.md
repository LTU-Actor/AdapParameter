Self-Adaptable Parameter Tuning
===============================

System Definition
-----------------

This system will allow ROS nodes to register with a parameter server similar to [dynamic reconfigure](http://wiki.ros.org/dynamic_reconfigure), but will instead automatically attempt to optimize the parameters to meet some predefined goals.

#### Project Goals:

1. Decrease the number of tunable parameters without hurting flexibility
2. Make the parameters that we tune the output we want, not the inputs we
   currently have
3. Gather a measurable performance metric from the system
4. Implement both a conventional and neural-network based learner

System Functions
----------------

1. Register clients based on their node name
2. "Learn" the connections between a node's feedback and parameters
4. Save these profiles to disk
5. Push changes to a node's parameters to cause the node's feedback to match
   goals
4. Allow a node to dynamically update it's goals

Connections Bubble
------------------

```text

        ┌─────────────┐
        │     ROS     │
        └─────────────┘
       /               \
┌─────────────┐ ┌─────────────┐
│    Server   │-│   Client    │
└─────────────┘ └─────────────┘
       |
┌─────────────┐
│ Filesystem  │
└─────────────┘
```

Interface
---------

This system will implement a parameter server that treats nodes as "black
boxes", and will learn how to tune them. The learned profile (NN
weights/conventional algorithm parameters) will be saved to disk under a unique
hash of the number of input/output parameters and the node name.

#### Theoretical Communication

```text
┌───────────────────────┐         ┌────────────────────────┐
│ Server                │         │ Client                 │
│                       │         │                        │
│  register node      <------------- init communication    │
│                       │         │                        │
│  load learn file    <------------- send feedback         │
│  init parameters      │         │                        │
│  send parameters    -------------> update parameters     │
│                       │         │  iterate node          │
│  iterate learn algo <------------- send feedback         │
│  guess optimal params │         │                        │
│  send parameters    -------------> update parameters     │
│  ...                  │         │  ...                   │
└───────────────────────┘         └────────────────────────┘
```

### Service Types:

#### Register.srv

Each 'Tunable Node' will call a ROS Service from the Parameter server to
register itself. This request will include the following:
 - List of tune parameter names
 - Names of feedback parameters
 - Feedback parameter goals
 - Tuning frequency

On success, the parameter server will begin sending Tune Requests.

#### Tune.srv

The parameter server will attempt to service each node as fast as the
specified, but may limit this frequency to share performance with other nodes.
The server must also wait on feedback if it does not come fast enough.  Each
request will contain a new set of parameter values. The

If a service request fails, the parameter server will query the health of the
Node from the ROS core. If the node has exited or re-spawned, then the server
will automatically unregister the node to be ready for a new connetion. If the
node is still alive but not responding, then the parameter server will wait a
predefined amount of time before making another Tune Request and repeating the
process.

#### Feedback.srv

After a node receives a Tune Request, the node promises to respond with
feedback as soon as it is available. If a node fails to send the Feedback, the
server will wait a predefined amount of time before repeating the Tune Request,
then the next round the node will be automatically unregistered from the server.
This feedback is send as a separate service call to keep the message passing
linear, and much simpler to implement.

#### UpdateGoals.srv

At and point, the Node being tuned may request that its goals be updated. This
service must be called by the node itself.

Test Cases
----------

[ROSUnitTesting](http://wiki.ros.org/action/show/Quality/Tutorials/UnitTesting?action=show&redirect=UnitTesting)
will be used.

#### Level 1: Library Level

 - The Client library will be tested
   - exposes the Tune topic in the correct locations
   - successfully sends the correct Register service call
   - multiple calls to the library to not cause issue

#### Level 2: ROS Node Unit Test

 - The server node as a whole will be tested
   - Many connections at the same time
   - Many incorrect register subscriptions
   - Slow updates
   - Updating a client's goals

#### Level 3: ROS Nodes Integration/Regression Level

 - Example nodes will be provided to ensure the server correctly tunes them.
   They will all be tested at the same time.
   - 1 parameter / 1 feedback simple linear node
   - 1 parameter / 1 feedback complex polymorphic node
   - 3 parameter / 1 feedback complex polymorphic node
   - 1 parameter / 3 feedback complex polymorphic node
   - Camera brightness correction node
