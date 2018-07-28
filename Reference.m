classdef  Reference < handle
%Reference - a handle class with basic struct-like assignment capabilities.
%
%  obj=Reference(inputStruct)
%
%will turn an ordinary structure inputStruct into a handle object 
%having much the same behavior as the original struct, i.e., fields can be 
%dynamically added to it via
%
%   obj.field1=...;
%   obj.field2=...;
%   etc...
%
%However, because it is a handle object, the field data it carries gets passed
%around by reference.
%
%This is useful, for example, if we want a variable X to be processed by 
%reference in a function call, i.e.,
%
%  obj=Reference;
%  obj.X=X;
%  clear X
%  func(obj,...)
%
%would allow func() to process obj.X arbitrarily, without making a 2nd deep
%copy of X, and so that obj.X  in the base workspace would feel the 
%changes made by func.
    properties 
       data;
    end
    methods
        function obj=Reference(dataInput)
            if nargin==0, return; end
            obj.data=dataInput;
        end
        function out=subsref(obj,S)
            if isequal(S(1).type,'.')
                out=builtin('subsref',obj.data,S);
            else
                disp 'Indexing method unimplemented'
                keyboard
            end
        end
        function obj=subsasgn(obj,S,rhs)
            if isequal(S(1).type,'.')
                obj.data=builtin('subsasgn',obj.data,S,rhs);
            else
                disp 'Indexing method unimplemented'
                keyboard
            end
        end    
        function obj=display(obj)
             lbl=inputname(1);
             if isempty(lbl), lbl='ans'; end
             lbl=[lbl ' ='];
             T=evalc('obj.data');
             T=strrep(T,'ans =', lbl);
              jj=find(T~=sprintf('\n'),1,'last');
              T=T(1:jj);
              disp(T), disp ' '
        end  
    end
end