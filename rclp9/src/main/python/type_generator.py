#!/usr/bin/env python3

import os
import glob
import re

ARRAY_IS_UNBOUNDED = 0
ARRAY_IS_BOUNDED = 1
ARRAY_IS_FIXED = 2
IS_NOT_ARRAY = 3
pattern_array_type = re.compile(r'(\[(<=)?(\d+)?\])?')
def get_array_type(token):
    if token == None:
        return IS_NOT_ARRAY, -1
    
    tokens = pattern_array_type.match(token)
    bounded = tokens.group(2)
    size = tokens.group(3)
    
    if bounded == None and size == None:
        return ARRAY_IS_UNBOUNDED, -1
    
    if bounded == '<=' and size != None:
        return ARRAY_IS_BOUNDED, int(size)
    
    if bounded == None and size != None:
        return ARRAY_IS_FIXED, int(size)
    
    print('Warning: unknown type of array definition, will use java.util.ArrayList')
    return ARRAY_IS_UNBOUNDED

VAR_IS_UNBOUNDED = 0
pattern_bound = re.compile(r'(<=(\d+))?')
def get_bound(token):
    if token == None:
        return VAR_IS_UNBOUNDED
    
    tokens = pattern_bound.match(token)
    return int(tokens.group(2))

# type [length of string] [[length of array]] name [=] [initial value] [comment]
pattern_token = re.compile(r'^\s*([a-zA-Z0-9_]+)(\s*<=\s*\d+\s*)?(\[[0-9<=]*\])?\s+([a-zA-Z0-9_]+)\s*(=)?\s*([^#].*)?\s*(#.*)?')
def get_tokens(line):
    if re.match(r'^#', line) or len(line) == 0:
        return
    
    return pattern_token.match(line)

def get_type(token):
    if token == '':
        print('Error: missing type argument\n')
        return
    
    if token == 'bool':
        return 'boolean'
    
    if token == 'byte':
        return 'byte'
    
    if token == 'char':
        return 'char'
    
    if token == 'float32':
        return 'float'
    
    if token == 'float64':
        return 'double'
    
    if token == 'int8' or token == 'uint8':
        print('Warning: {} is not supported, will use byte instead\n'.format(token))
        return 'byte'
    
    if token == 'int16':
        return 'short'
    
    if token == 'uint16':
        print('Warning: uint16 is not supported, will use short instead')
        return 'short'
    
    if token == 'int32':
        return 'int'
    
    if token == 'uint32':
        print('Warning: uint32 is not supported, will use int instead')
        return 'int'
    
    if token == 'int64':
        return 'long'
    
    if token == 'uint64':
        print('Warning: uint64 is not supported, will use long instead')
        return 'long'
    
    if token == 'string':
        return 'String'
    
    if token == 'wstring':
        print('Warning: wstring is not supported, will use java.lang.String instead')
        return 'String'
    
    # this type should be defined somewhere else
    return token

pattern_initializer = re.compile(r'\[(.*)\]')
pattern_quote = re.compile(r'(\'(.*)\')?')
def get_value(token, java_type):
    if token == None:
        return ''
    
    tokens = pattern_initializer.match(token)
    if tokens != None:
        return tokens.group(1)
    
    if java_type != 'String':
        return token
    
    tokens = pattern_quote.match(token)
    if tokens != None:
        return '\"{}\"'.format(tokens.group(2))
    else:
        return token

def get_wrapper(type):
    if type == 'byte':
        return 'Byte'
    
    if type == 'short':
        return 'Short'
    
    if type == 'int':
        return 'Integer'
    
    if type == 'long':
        return 'Long'
    
    if type == 'float':
        return 'Float'
    
    if type == 'double':
        return 'Double'
    
    if type == 'char':
        return 'Character'
    
    if type == 'boolean':
        return 'Boolean'
    
    # this type is not a primitive
    return type

def is_constant(equal, val):
    if equal == '=':
        if val == '':
            print('Warning: constant definition requires initializer, will define as a variable')
        else:
            return True
    return False

def compose(is_const, java_type, bounded, array_type, array_size, var_name, val):
    if array_type == IS_NOT_ARRAY:
        if bounded > 0:
            print('Warning: bounding type is not supported, will use unbounded type instead')
        
        if val != '':
            str = '{} {} = {};'.format(java_type, var_name, val)
            if is_const:
                return 'final {}'.format(str), False
            else:
                return str, False
        else:
            return '{} {};'.format(java_type, var_name), False
        
    if array_type == ARRAY_IS_BOUNDED or array_type == ARRAY_IS_FIXED:
        if bounded > 0:
            print('Warning: bounding type is not supported, will use unbounded type instead')
        
        if is_const:
            print('Warning: unknown constant array definition, will define as a variable')
        elif val != '':
            print('Warning: unknown array definition, will define as an uninitialized array')
            
        return '{}[] {} = new {}[{}];'.format(java_type, var_name, java_type, array_size), False
    
    if array_type == ARRAY_IS_UNBOUNDED:
        if bounded > 0:
            print('Warning: bounding type is not supported, will use unbounded type instead')
        
        wrapper = get_wrapper(java_type)
        if val != '':
            str = '{}[] {} = {{{}}};'.format(java_type, var_name, val)
            if is_const:
                return 'final {}'.format(str), False
            else:
                return str, False
        else:
            return 'ArrayList<{}> {} = new ArrayList<{}>();'.format(wrapper, var_name, wrapper), True

        
    print('Error: unknown definition of variable')
    return ''

def parse(tokens):
    java_type = get_type(tokens.group(1))
    bounded = get_bound(tokens.group(2))
    array_type, size = get_array_type(tokens.group(3))
    var_name = tokens.group(4)
    val = get_value(tokens.group(6), java_type)
    is_const = is_constant(tokens[5], val)
    
    str, arrayList_used = compose(is_const, java_type, bounded, array_type, size, var_name, val)
        
    return str, arrayList_used

def main():
    java_dir = './src/main/java/rclp9/msg'
    if not os.path.exists(java_dir):
        os.makedirs(java_dir)
    
    rsc_dir = './src/main/resources/msg/'
    files = glob.glob(os.path.join(rsc_dir, '*.msg'))
    for file in files:
        package_name = 'rclp9.msg';
        output_str = 'package {};\n\n'.format(package_name)
        class_name = os.path.splitext(os.path.basename(file))[0]
        class_def_str = 'public class {} {{\n'.format(class_name)
        
        print('Processing {} ...'.format(file))
        f = open(file, 'r')
        lines = f.readlines()
        f.close()
        
        body = ''
        import_arrayList = False
        for l in lines:
            tokens = get_tokens(l.strip())
            if tokens == None:
                continue
            
            generated, arrayList_used = parse(tokens)
            body += '\t{}\n'.format(generated)
            import_arrayList = import_arrayList or arrayList_used
        
        if import_arrayList:
            output_str += 'import java.util.ArrayList;\n\n'
        
        output_str += '{}{}}}\n'.format(class_def_str, body)
        
        output_file = '{}/{}.java'.format(java_dir, class_name)
        f = open(output_file, 'w')
        f.write(output_str)
        f.close()

if __name__ == "__main__":
    main()