/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    // STENCIL: implement your min binary heap insert operation
    heap.push(new_element);
    shiftUp(heap, heap.length - 1);

    function shiftUp(heap, index) {
        if (index <= 0) {
            return;
        }

        var parent = Math.floor((index - 1) / 2);
        if (heap[index] < heap[parent]) {
            var tmp = heap[index];
            heap[index] = heap[parent];
            heap[parent] = tmp;
            shiftUp(heap, parent);
        }
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    if (heap.length <= 0) {
        return;
    }
    if (heap.length === 1) {
        return heap.pop();
    }

    var min = heap[0];
    heap[0] = heap.pop();
    shiftDown(heap, 0);
    return min;

    function shiftDown(heap, root) {
        var left = root * 2 + 1;
        var right = (root + 1) * 2;
        var minIndex = root;
        if (left < heap.length && heap[left] < heap[minIndex]) {
            minIndex = left;
        }
        if (right < heap.length && heap[right] < heap[minIndex]) {
            minIndex = right;
        }
        if (minIndex !== root) {
            var tmp = heap[root];
            heap[root] = heap[minIndex];
            heap[minIndex] = tmp;
            shiftDown(heap, minIndex);
        }
    }
}

// assign extract function within minheaper object
    // STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;





