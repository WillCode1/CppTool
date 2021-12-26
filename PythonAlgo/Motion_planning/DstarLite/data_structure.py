import heapq


class PriorityQueue:
    """
    A priority queue optimized for the D* Lite algorithm. Implements O(1) lazy
    random element deletion, and O(1) contains, with O(1) average time dequeuing.
    This allows the algorithm to scale to large numbers of obstacles.
    """

    def __init__(self):
        """
        Initialize the backing heap and the presence hash used for lazy delete.
        """
        self._heap = []
        self._present_elems = {}
        self._size = 0

    # Enqueues an element in the PQ.
    def enqueue(self, elem):
        # Add the element to the heap, increment the size, and set the presence
        # of the given elements vertex to True.
        # heapq.heappush(self._heap, (priority, point))
        heapq.heappush(self._heap, elem)

        self._present_elems[elem[1]] = True
        self._size += 1

    # Dequeues the lowest element from the PQ.
    def dequeue(self):
        while len(self._heap) > 0:
            # Continue popping elements from the queue until we get one
            # that actually should be in the queue.
            elem = heapq.heappop(self._heap)

            if self._present_elems[elem[1]]:
                # Decrement the size, since this element was actually in the queue.
                self._size -= 1
                self._present_elems[elem[1]] = False
                return elem

            self._present_elems[elem[1]] = False

    def remove(self, elem):
        """
        O(1) lazy delete remove from the queue, given a vertex.
        """

        # Set the presence of the vertex in the queue to false, and reduces
        # the size.
        if self._present_elems.get(elem, False):
            self._present_elems[elem] = False
            self._size -= 1

    def peek(self):
        """
        Gets the top of the PQ without removing it
        """
        while len(self._heap) > 0:
            # Pop any elements that are not actually in the queue, and return
            # the first one that hasn't been lazy deleted.
            elem = self._heap[0]

            if self._present_elems[elem[1]]:
                return elem

            heapq.heappop(self._heap)

    def __len__(self):
        return self._size

    def __contains__(self, elem):
        return self._present_elems.get(elem[1], False)
