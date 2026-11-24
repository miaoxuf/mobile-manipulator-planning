classdef MinHeap < handle
%MINHEAP Binary min-heap of (key, value) pairs with numeric keys.
%   h = MinHeap(capacity) creates an empty heap; storage grows as needed.
%   h.push(key, value), [key, value] = h.pop(), h.is_empty(), h.count.
%   Used as the priority queue of the platform planners (lazy deletion).

    properties (SetAccess = private)
        count = 0
    end
    properties (Access = private)
        keys
        vals
    end

    methods
        function h = MinHeap(capacity)
            if nargin < 1, capacity = 64; end
            h.keys = zeros(max(capacity, 1), 1);
            h.vals = zeros(max(capacity, 1), 1);
        end

        function push(h, key, val)
            n = h.count + 1;
            if n > numel(h.keys)
                h.keys(2 * n) = 0;
                h.vals(2 * n) = 0;
            end
            i = n;
            while i > 1
                p = floor(i / 2);
                if h.keys(p) <= key
                    break
                end
                h.keys(i) = h.keys(p);
                h.vals(i) = h.vals(p);
                i = p;
            end
            h.keys(i) = key;
            h.vals(i) = val;
            h.count = n;
        end

        function [key, val] = pop(h)
            assert(h.count > 0, 'MinHeap:empty', 'pop from an empty heap');
            key = h.keys(1);
            val = h.vals(1);
            lastK = h.keys(h.count);
            lastV = h.vals(h.count);
            n = h.count - 1;
            h.count = n;
            if n == 0
                return
            end
            i = 1;
            while true
                c = 2 * i;
                if c > n
                    break
                end
                if c < n && h.keys(c + 1) < h.keys(c)
                    c = c + 1;
                end
                if h.keys(c) >= lastK
                    break
                end
                h.keys(i) = h.keys(c);
                h.vals(i) = h.vals(c);
                i = c;
            end
            h.keys(i) = lastK;
            h.vals(i) = lastV;
        end

        function tf = is_empty(h)
            tf = h.count == 0;
        end
    end
end
