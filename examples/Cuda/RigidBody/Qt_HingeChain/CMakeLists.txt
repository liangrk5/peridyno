if(PERIDYNO_LIBRARY_RIGIDBODY)
    set(LIB_DEPENDENCY 
        RigidBody
        Modeling
        QtGUI)
    add_example(Qt_HingeChain RigidBody LIB_DEPENDENCY)
endif()