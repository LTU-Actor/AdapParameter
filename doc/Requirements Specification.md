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

Interface
---------

This system will implement a parameter server that treats nodes as "black
boxes", and will learn how to tune them. The learned profile (NN
weights/conventional algorithm parameters) will be saved to disk under a unique
hash of the number of input/output parameters and the node name.

#### Protocol

```text
+--------+              +------+           +----------+
| Server |              | Node |           | ROS Core |
+--------+              +------+           +----------+
    |         registration  |                    |
    | <--------------------<|                    |
    |                       |                    |
    |  request tune         |                    |
    |>--------------------> |                    |
    |  request tune         |                    |
    |>--------------------> |                    |
    |                       |                    |
    |         update goals  |                    |
    | <--------------------<|                    |
    |                       |                    |
    |  request tune         |                    |
    |>--------------------> |                    |
    |  request tune         |                    |
    |>--------------------> |                    |
    |                       X Exit               |
    |  request tune                              |
    |>------------------------/ Fail /---------> |
    |                                            |
    |  check if 'Node' is alive                  |
    |>-----------------------------------------> |
    |                                            |
    X Exit or Wait on 'Node'                     |
                                                 V
```


#### Registration

Each 'Tunable Node' will call a ROS Service from the Parameter server to
register itself. This request will include the following:
 - Number of tune parameters
 - List of tune parameter names
 - Number of feedback parameters
 - Names of feedback parameters
 - Feedback parameter goals
 - Tuning frequency

On success, the parameter server will begin servicing Tune Requests.

#### Tune Request

The parameter server will attempt to service each node as fast as the
specified, but may limit this frequency to share performance with other nodes.
Each request will contain a new set of parameter values, and will return the
last request's feedback values. After each iteration, the parameter server will
evaluate the feedback parameters and continue tuning.

If a service request fails, the parameter server will query the health of the
Node from the ROS core. If the node has exited or re-spawned, then the server
will automatically unregister the node to be ready for a new connetion. If the
node is still alive but not responding, then the parameter server will wait a
predefined amount of time before making another Tune Request and repeating the
process.

#### Update Goals

At and point, the Node being tuned may request that its goals be updated.
