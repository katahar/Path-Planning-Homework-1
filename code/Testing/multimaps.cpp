#include <iostream>
#include <queue>
#include <vector>
#include <map>
#include <iterator>


class customClass
{
    public:

        double evaluation_metric = 100;
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

        customClass* get_ptr()
        {
            // std::cout << "Getting ptr" << std::endl;
            return this;
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

void print_table(std::multimap<double, customClass*> hash_table_mm)
{
    std::cout << "F Value" << "\t\t" << "letter" << "\t" << "eval_metric" << "\t" << "other_metric" << "\n";

    for (auto itr = hash_table_mm.begin(); itr != hash_table_mm.end(); itr++)
    {
        std::cout << itr->first << "\t ";
        itr->second->print();
    }
}

void remove_value(std::multimap<double, customClass*> hash_table_mm_in, std::multimap<double, customClass*>::iterator input)
{
    std::multimap<double, customClass*>::iterator temp = hash_table_mm_in.find(input->first);
    if(temp != hash_table_mm_in.end())
    {
        for(std::multimap<double, customClass*>::iterator itr = temp; itr->first == input->first; itr++)
        {
            if(itr->second == input->second)
            {
                std::cout << "Found the search iterator!" << std::endl;
                hash_table_mm_in.erase(itr);
                break;
            }
        }
        std::cout << "New table with removed value: " << std::endl;
        print_table(hash_table_mm_in);
    }
    else
    {
        std::cout << "Value not found :(  " << std::endl;

    }
}

void remove_value(std::multimap<double, customClass*> hash_table_mm_in, double key, customClass* nodeptr)
{
    std::multimap<double, customClass*>::iterator temp = hash_table_mm_in.find(key);
    if(temp != hash_table_mm_in.end())
    {
        for(std::multimap<double, customClass*>::iterator itr = temp; itr->first == key; itr++)
        {
            if(itr->second == nodeptr)
            {
                std::cout << "Found the search iterator!" << std::endl;
                hash_table_mm_in.erase(itr);
                break;
            }
        }
        std::cout << "New table with removed value: " << std::endl;
        print_table(hash_table_mm_in);
    }
    else
    {
        std::cout << "Value not found :(  " << std::endl;

    }
}

main()
{
    customClass instA = customClass( 5, 1, 'A');
    customClass instB = customClass( 1, 2, 'B');
    customClass instC = customClass( 3, 3, 'C');
    customClass instD = customClass( 3, 4, 'D');
    customClass instE = customClass( 7, 5, 'E');
    customClass instF = customClass( 1 ,6, 'F');


    std::multimap<double, customClass*> hash_table_mm;
    hash_table_mm.insert(std::pair<double,customClass*>(instA.evaluation_metric,instA.get_ptr()));
    hash_table_mm.insert(std::pair<double,customClass*>(instB.evaluation_metric,instB.get_ptr()));
    hash_table_mm.insert(std::pair<double,customClass*>(instC.evaluation_metric,instC.get_ptr()));
    hash_table_mm.insert(std::pair<double,customClass*>(instD.evaluation_metric,instD.get_ptr()));
    hash_table_mm.insert(std::pair<double,customClass*>(instE.evaluation_metric,instE.get_ptr()));
    hash_table_mm.insert(std::pair<double,customClass*>(instF.evaluation_metric,instF.get_ptr()));

    print_table(hash_table_mm);
   
   // Using pointer arguement
    // auto delet_val = hash_table_mm.begin();
    // delet_val++;
    // delet_val++;
    // delet_val++;

    // std::cout << "removing: ";
    // delet_val->second->print();

    // remove_value(hash_table_mm, delet_val);


    //using node* and key arguement
    remove_value(hash_table_mm, 3, instD.get_ptr());





}   
