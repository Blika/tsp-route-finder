#include "path/path_finder.hpp"

#include <iostream>
#include <stdexcept>

int main(){
    try{
        PathFinder pf{};
        pf.run();        
    }catch(const std::exception& e){
		std::cerr << e.what() << '\n';
	}
    return 0;
}