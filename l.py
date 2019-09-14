import collections

n = eval(input())
a = list(map(int, input().split()))

lw = collections.Counter()
l_count = 0
rw = collections.Counter(a)
r_count = n
res = [0] * (n - 1)
for i in range(n - 1):
    lw[a[i]] += 1
    l_count += 1
    rw[a[i]] -= 1
    r_count -= 1
    res[i] = res[i - 1] + (r_count - rw[a[i]]) - (l_count - lw[a[i]])
    # 第0次循环不需要单独加判断，因为res[-1]=res[0]=0

print(' '.join(map(str, res)))
