#include "stringutil.h"

std::vector<std::string> str_explode(const std::string& text, const std::string& separators)
{
        std::vector<std::string> words;
        size_t n     = text.length ();
        size_t start = text.find_first_not_of (separators);

        while (start < n)
        {
                size_t stop = text.find_first_of (separators, start);
                if (stop > n) stop = n;
                words.push_back (text.substr (start, stop-start));
                start = text.find_first_not_of (separators, stop+1);
        };
        return words;
}
