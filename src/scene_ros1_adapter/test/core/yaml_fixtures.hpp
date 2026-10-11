// Generated once with PyYAML 5.3.1 CSafeDumper; see the package report.
static const YamlFixture kYamlFixtures[] = {
  {"quoted_scalars", R"JSON({"yes": "yes", "numeric-looking": "1e3", "null": "null", "leading": "  value", "trailing": "value  ", "unicode": "café Ω 🌍", "syntax": "hello: world", "hash": "hi # comment", "apostrophe": "it's true", "empty": ""})JSON", R"YAML('yes': 'yes'
numeric-looking: 1e3
'null': 'null'
leading: '  value'
trailing: 'value  '
unicode: "café Ω \U0001F30D"
syntax: 'hello: world'
hash: 'hi # comment'
apostrophe: it's true
empty: ''
)YAML"},
  {"folded_plain", R"JSON({"name": "long string long string long string long string long string long string long string long string long string long string long string long string long string long string long string"})JSON", R"YAML(name: long string long string long string long string long string long string long
  string long string long string long string long string long string long string long
  string long string
)YAML"},
  {"folded_single", R"JSON({"name": "yes: long string long string long string long string long string long string long string long string long string long string long string long string long string long string long string"})JSON", R"YAML(name: 'yes: long string long string long string long string long string long string
  long string long string long string long string long string long string long string
  long string long string'
)YAML"},
  {"folded_double", R"JSON({"name": "long string long string long string long string long string long string long string long string long string long string long string long string long string long string long string\t end"})JSON", R"YAML(name: "long string long string long string long string long string long string long
  string long string long string long string long string long string long string long
  string long string\t end"
)YAML"},
  {"float_forms", R"JSON({"numbers": [0.0, -0.0, 1.0, 1e-07, 1e-06, 1e-05, 0.0001, 1000000000000000.0, 1e+16, 1e+20, 1e+23, 1.2345678901234567, 1.7976931348623157e+308, 5e-324, 2.2250738585072014e-308]})JSON", R"YAML(numbers:
- 0.0
- -0.0
- 1.0
- 1.0e-07
- 1.0e-06
- 1.0e-05
- 0.0001
- 1000000000000000.0
- 1.0e+16
- 1.0e+20
- 1.0e+23
- 1.2345678901234567
- 1.7976931348623157e+308
- 5.0e-324
- 2.2250738585072014e-308
)YAML"},
  {"escape_forms", R"JSON({"values": ["a\nb", "a\nb\n", "a\n\n\nb", "\t", "\u0000", "a \nb", "a\n b", "", " ", " ", " ", "﻿"]})JSON", R"YAML(values:
- 'a

  b'
- 'a

  b

  '
- 'a



  b'
- "\t"
- "\0"
- "a \nb"
- "a\n b"
- "\N"
-  
- '   '
- '   '
- "\uFEFF"
)YAML"},
  {"collections", R"JSON({"a": [], "b": {}, "c": null, "d": true, "e": false, "nested": [[1, 2], {"x": 1, "y": 2}]})JSON", R"YAML(a: []
b: {}
c: null
d: true
e: false
nested:
- - 1
  - 2
- x: 1
  y: 2
)YAML"},
  {"string_resolvers", R"JSON({"values": ["Y", "yes", "NO", "on", "false", "null", "~", "1e3", "1e+3", "1.0e3", "1.0e+3", "012", "0o12", "0x10", "0b10", "1:02", "1:02.5", "2001-12-15", "2001-12-15T02:59:43.1Z", "<<", "=", "---text", "...text", "?word", ":word", "-word", "@word", "[x]"]})JSON", R"YAML(values:
- Y
- 'yes'
- 'NO'
- 'on'
- 'false'
- 'null'
- '~'
- 1e3
- 1e+3
- 1.0e3
- '1.0e+3'
- '012'
- 0o12
- '0x10'
- '0b10'
- '1:02'
- '1:02.5'
- '2001-12-15'
- '2001-12-15T02:59:43.1Z'
- '<<'
- '='
- '---text'
- '...text'
- ?word
- :word
- -word
- '@word'
- '[x]'
)YAML"},
  {"long_key", R"JSON({"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa": "value"})JSON", R"YAML(aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa: value
)YAML"},
  {"multiline_key", R"JSON({"a\nb": "value"})JSON", R"YAML(? 'a

  b'
: value
)YAML"},
};
