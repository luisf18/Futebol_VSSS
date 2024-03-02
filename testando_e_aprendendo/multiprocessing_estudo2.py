import multiprocessing as mp
import concurrent.futures
import time


def f( name, n ):
    print('Pais: ', name)
    return f'->> Pais[{n}]: {name} [{time.time()}]'

if __name__ == "__main__":
    
	with concurrent.futures.ProcessPoolExecutor() as executor:
         c1 = executor.submit( f, 'Brasil', 1 )
         c2 = executor.submit( f, 'Italia', 2 )
         print(c1.result())
         print(c2.result())

	with concurrent.futures.ProcessPoolExecutor() as executor:
         ct = [ 'Brasil', 'Italia' ]
         c2 = [ 1, 2 ]
         c = executor.map( f, ct, c2 )
         for x in c:
            print(x)