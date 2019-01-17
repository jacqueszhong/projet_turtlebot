import matplotlib.pyplot as plt
import numpy as np

def normalise(P):
	""" Normalisation du polynome: le premier coefficient
	correspond alors au coefficient dominant """
	i = 0
	try:
		while P[i] == 0:
			i+=1
		return P[i:]
	except:
		return []

def evalue(P,x):
	""" evalue P(x) par la methode de Horner """
	if P == []:
		return 0
	u = P[0]
	for i in range(1,len(P)):
		u = u * x + P[i]
	return u

def mult(P,a):
	""" Multiplication d un polynome par un scalaire """
	if a == 0:
		return []
	return ([a * b for b in P])

def somme(P,Q):
	""" Somme de deux polynomes """
	p = len(P)
	q = len(Q)
	if p< q:
		P = [0] * (q-p) + P
	else:
		Q = [0] * (p-q) + Q
	S = [ sum(C) for C in zip(P,Q)]
	return normalise(S)

def produit(P,Q):
	""" Produit de deux polynomes """
	P = P[:]
	Q = Q[:]
	R = [0] * (len(P)+len(Q)-1)
	for (i,a) in enumerate(P):
		for (j,b) in enumerate(Q):
			R[i+j] += a * b
	return(R)


def Lagrange(nodes,i):
	""" i-ieme polynome de Lagrange associe a la liste noeuds """
	L = [1]
	X = nodes[:]
	y = X.pop(i)
	for x in X:
		l=produit(L,[1,-x])
		d=float(1.0 / (y-x))
		L = mult(l,d)
	return(L)

def interpole(X,Y):
	""" Polynome interpolateur aux points (X,Y) """
	L = []
	for i, y in enumerate(Y):
		L = somme(L,mult(Lagrange(X,i),y))
	return(L)

if __name__ == '__main__':
	p=[(66, 108), (232, 255), (263, 50), (309, 57), (318, 247), (358, 246), (389, 29)]
	p=[(0,2.5),(-1,1.5),(0,0),(3,0)]
	x = [coord[0] for coord in p]
	y = [coord[1] for coord in p]
	print("x="+str(x))
	print("y="+str(y))
	T = range(1,len(Y)+1)
	X=interpole(T,x)
	Y=interpole(T,y)
	print("X="+str(X))
	print("Y="+str(Y))

	path=[]
	for i in np.arange(1,len(p)+1,1):
		path.append((evalue(X,i),evalue(Y,i)))
	print(path)
	xval = [coord[0] for coord in path]
	yval = [coord[0] for coord in path]

	plt.plot(x, y, "ro")
	plt.plot(xval,yval)

	plt.show()

