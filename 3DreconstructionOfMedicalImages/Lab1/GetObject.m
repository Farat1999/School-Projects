function [ obj, index ] = GetObject( objList, objName )
% ------------------------------------------------------------------------	
%																			
% FUNCTION: GetObject()														
%																			
% synopsis:
%       obj = GetObject( objList, objName )
%																			
% DESCRIPTION:																
%	Looks for an Object by its name.										
%																			
% Arguments:																
%	objList - list of all the objects (i.e. struct array).								        
%	objName  -	name of the requested object.							            
%																			
% Results:																	
%	The object found in the list (i.e. struct).	If not found, returns an
%	empty object ([]).
%																			
% Remarks:	The search is case-insensitive.		
%																			
% ------------------------------------------------------------------------	

if nargin <2
   error ('GetObject: incorrect number of arguments.');
end

if isempty(objList)==1
   obj = [] ;
   index = [] ;
   return ;
end

index = 0;
for i=1:length(objList)
   if (strcmpi(deblank(objList(i).name), deblank(objName))==1)
      index = i;
   end
end
if index==0
   obj = [] ;
else
   obj = objList(index);
end

