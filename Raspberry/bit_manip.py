

def get_talk_bytes_from_command(cmd):
    
    cmd[1] += 1024
    cmd[2] += 1024

    def make_number_():
        i = 0
        for nb, v in zip(bits_reserved, cmd):
            i = i << nb
            i = i | v

        return i 

    def make_bytes_(i):
    
        vals = [0]*nbytes
        for k in range(nbytes):
            # First byte is the msot significant.
            vals[-k-1] = i & 255
            i = i>>8

        return bytearray(vals)

    return make_bytes_(make_number_())

def depack(byte_arr, bits_reserved):

    n1 = byte_arr[0] >> 6
    n2 = ((byte_arr[0] & 63)<<5) | (byte_arr[1]>>3)
    n3 = (byte_arr[1] & 7)<<8 | byte_arr[2] 

    return n1, n2-1024, n3-1024

nbytes = 3 
cmd = [0, -1024, 1023]
bits_reserved = [2, 11, 11]
byte_arr = get_talk_bytes_from_command(cmd)
print(byte_arr)
print(depack(byte_arr, bits_reserved))
    

