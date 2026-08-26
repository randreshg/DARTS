#include <iostream>

#include "darts.h"

using namespace darts;

class HelloCodelet : public Codelet
{
public:
    HelloCodelet(ThreadedProcedure *tp)
        : Codelet(0, 0, tp, SHORTWAIT)
    {
    }

    void fire(void) override
    {
        std::cout << "Hello, world from DARTS!" << std::endl;
        Runtime::finalSignal.decDep();
    }
};

class HelloTP : public ThreadedProcedure
{
public:
    HelloCodelet hello;

    HelloTP()
        : ThreadedProcedure(), hello(this)
    {
        add(&hello);
    }
};

int main()
{
    ThreadAffinity affinity(1U, 1U, COMPACT, TPDYNAMIC, MCDYNAMIC);
    if (!affinity.generateMask())
        return 1;

    Runtime(&affinity).run(launch<HelloTP>());
    return 0;
}
