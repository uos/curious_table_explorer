namespace utils {

/* provide custom static_cast<>() implementations
 * for types not in the domain of this project.
 *
 * required conversions are implemented in src/convert.cpp
 * Implementations mustn't do anything unexpected.
 *
 * usage: `NewType a = convert<NewType>(expression)`
 *        If the compiler can't infer the correct type of `expression` use:
 *        `NewType a = convert<NewType,OldType>(expression)`
 *        which is functionally equivalent to
 *        `NewType a = convert<NewType>(static_cast<OldType>(expression))`
 */
template <class T, class U> T convert(const U&);

}
