
class StringArray:

    def __init__(self, 
            delimiter=";"):
        
        self.delimiter = delimiter

    def encode(self, strings):

        """
        Encodes a list of strings into a single string using the delimiter.
        :param strings: List[str] - The list of strings to encode.
        :return: str - The encoded string.
        """
        return self.delimiter.join(strings)

    def decode(self, encoded_string):
        """
        Decodes a string back into a list of strings using the delimiter.
        :param encoded_string: str - The encoded string.
        :return: List[str] - The list of decoded strings.
        """
        return encoded_string.split(self.delimiter)