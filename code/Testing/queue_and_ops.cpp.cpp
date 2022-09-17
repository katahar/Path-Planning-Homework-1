#include <iostream>
#include <queue>
#include <vector>

class customClass
{
    public:
        int evaluation_metric = 100;
        int other_metric = 2;
        char let; 
        
        customClass(int eval, int other, char inLet)
        {
            evaluation_metric = eval; 
            other_metric = other;
            let = inLet; 
        }
        
        customClass()
        {
            
        }

        void print()
        {
            std::cout << "\t" << let << "\t" << evaluation_metric << "\t\t" << other_metric << "\n";
        }

        friend bool operator<(const customClass& l, const customClass& r)
        {
            return l.evaluation_metric < r.evaluation_metric;
        }
        friend bool operator>(const customClass& l, const customClass& r)
        {
            return l.evaluation_metric > r.evaluation_metric;
        }
        friend bool operator==(const customClass& l, const customClass& r)
        {
            return l.evaluation_metric == r.evaluation_metric;
        }
};

void showpq(std::priority_queue<customClass, std::vector<customClass>, std::greater<std::vector<customClass>::value_type> > gq)
{
    std::cout << "\t" << "letter" << "\t" << "eval_metric" << "\t" << "other_metric" << "\n";

    std::priority_queue<customClass, std::vector<customClass>, std::greater<std::vector<customClass>::value_type> > g = gq;
    while (!g.empty()) {
        customClass print_val = g.top();
        std::cout << "\t" << print_val.let << "\t" << print_val.evaluation_metric << "\t\t" << print_val.other_metric << "\n";
        g.pop();
    }
    std::cout << "\n";
}

main()
{
    customClass instA = customClass( 5, 1, 'A');
    customClass instB = customClass( 1, 2, 'B');
    customClass instC = customClass( 4, 3, 'C');
    customClass instD = customClass( 3, 4, 'D');
    customClass instE = customClass( 7, 5, 'E');
    customClass instF = customClass(11 ,6, 'F');
    
    if(instA<instB)
    {
        printf("Incorrect\n");
    }
    if(instA>instB) 
    {
        printf("Correct Output\n");
    }
    if(instA==instB) 
    {
        printf("Incorrect Output\n");
    }
    printf("Less\n");

    std::priority_queue<customClass, std::vector<customClass>, std::greater<std::vector<customClass>::value_type> > pq;
    pq.push(instA);
    pq.push(instB);
    pq.push(instC);
    pq.push(instD);
    pq.push(instE);
    pq.push(instF);
    showpq(pq);


    std::vector<std::vector<std::vector<customClass>>> map_vec(3, std::vector<std::vector<customClass>>(4, std::vector<customClass>(5)));
    map_vec[0][1][1] = instA; 
    std::cout << "AA";
    map_vec[0][1][1].print();


}   
