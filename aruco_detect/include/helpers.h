#include <vector>
#include <string>

void split(std::vector<std::string> &res, const std::string &str, char delim)
{
  size_t start;
  size_t end = 0;
  while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
  {
    end = str.find(delim, start);
	res.push_back(str.substr(start, end - start));
  }
}