namespace curious_table_explorer {
namespace utils {

// provide conversion functions between arbitrary types
// required conversions are implemented in src/convert.cpp
// Implementations mustn't do anything unexpected.
template <class T, class U> T convert(const U&);

}
}
