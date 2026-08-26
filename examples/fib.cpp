#include <cstdlib>
#include <iostream>

#include "darts.h"

using namespace darts;

class FibTP;

class FibStartCodelet : public Codelet
{
public:
    FibStartCodelet(ThreadedProcedure *tp)
        : Codelet(0, 0, tp, SHORTWAIT)
    {
    }

    void fire(void) override;
};

class FibCombineCodelet : public Codelet
{
public:
    FibCombineCodelet(ThreadedProcedure *tp)
        : Codelet(2, 2, tp, LONGWAIT)
    {
    }

    void fire(void) override;
};

class FibTP : public ThreadedProcedure
{
public:
    int n;
    int left;
    int right;
    int *result;
    Codelet *completion;
    FibStartCodelet start;
    FibCombineCodelet combine;

    FibTP(int value, int *output, Codelet *done)
        : ThreadedProcedure(),
          n(value),
          left(0),
          right(0),
          result(output),
          completion(done),
          start(this),
          combine(this)
    {
        add(&start);
    }
};

void FibStartCodelet::fire(void)
{
    FibTP *fib = static_cast<FibTP *>(myTP_);

    if (fib->n < 2)
    {
        *fib->result = fib->n;
        fib->completion->decDep();
        return;
    }

    invoke<FibTP>(fib, fib->n - 1, &fib->left, &fib->combine);
    invoke<FibTP>(fib, fib->n - 2, &fib->right, &fib->combine);
}

void FibCombineCodelet::fire(void)
{
    FibTP *fib = static_cast<FibTP *>(myTP_);
    *fib->result = fib->left + fib->right;
    fib->completion->decDep();
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: fib_example N" << std::endl;
        return 1;
    }

    int n = std::atoi(argv[1]);
    if (n < 0)
    {
        std::cerr << "N must be non-negative" << std::endl;
        return 1;
    }

    int result = 0;
    ThreadAffinity affinity(1U, 1U, COMPACT, TPDYNAMIC, MCDYNAMIC);
    if (!affinity.generateMask())
        return 1;

    Runtime(&affinity).run(launch<FibTP>(n, &result, &Runtime::finalSignal));
    std::cout << "fib(" << n << ") = " << result << std::endl;
    return 0;
}
