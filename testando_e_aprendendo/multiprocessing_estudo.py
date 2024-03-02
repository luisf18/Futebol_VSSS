#import multiprocessing as mp
#import concurrent.futures
#import numpy as np

def print_func(continent, A ):
	print('The name of continent is : ', continent)
	A.put( f'The name of continent is : {continent}' )

if __name__ == "__main__":
     import time
     import multiprocessing as mp
     start = time.time()
     import numpy as np
     A = mp.Queue()
     B = np.zeros( (500,500,3), np.uint8 )
     A.put( B )
     print(A.get())
     print("Number of cpu : ", mp.cpu_count())
     names = ['America', 'Europe', 'Africa', 'USA']
     procs = []
     for name in names:
        proc = mp.Process(target=print_func, args=(name,A,))
        procs.append(proc)
        proc.start()
     for proc in procs:
        proc.join()
     while( not A.empty() ):
        print( A.get() )
     print( int(1000*(time.time() - start)),'ms' )
     
