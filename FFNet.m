classdef FFNet < handle

    properties
        
        N0;
        N1;
        N2;
        
        W1;
        W2;
        
        b1;
        b2;
        
        d2;
        d1;
        d0;
        
        r0;
        r1;
        r2;
    
    end
    
    methods
        
        function obj = FFNet(N0,N1,N2)
            
            obj.N0  = N0;
            obj.N1  = N1;
            obj.N2  = N2;
            
            obj.W2  = zeros(N2,N1);
            obj.W1  = 0.05*randn(N1,N0)/sqrt(N0);
            obj.b2  = 0.05*randn(N2,1);
            obj.b1  = 0.05*randn(N1,1);
                        
        end
        
        function [cst,grad] = FFCost(obj,theta,patterns,DataFactory)
            
            if nargin==4
                patterns = DataFactory();
            end
            
            M   = patterns{1};
            V   = patterns{2};
            
            N0  = obj.N0;
            N1  = obj.N1;
            N2  = obj.N2;
            
            W1 = reshape(theta(1:(N0*N1)), N1, N0);
            W2 = reshape(theta((N0*N1+1):(N0*N1+N1*N2)), N2, N1);
            b1 = theta((N0*N1+N1*N2+1):(N0*N1+N1*N2+N1));
            b2 = theta((N0*N1+N1*N2+N1+1):(N0*N1+N1*N2+N1+N2));
            
            W1grad  = zeros(size(W1)); 
            W2grad  = zeros(size(W2));
            b1grad  = zeros(size(b1)); 
            b2grad  = zeros(size(b2));
            
            nsamples    = size(M,2);
            x1          = W1*M + repmat(b1,1,nsamples);
            r1          = tanh(x1);
            x2          = W2*r1   + repmat(b2,1,nsamples);
            r2          = x2;

            %% Backpropagation
            d2 = (r2-V); 
            f2 = 1;
            e2 = d2.*f2;
            d1 = W2'*e2;
            f1 = 1-r1.^2;
            e1 = d1.*f1;
            d0 = W1'*e1;

            n2ns   = N2*nsamples;
            W2grad = e2*r1'/n2ns;
            W1grad = e1*M'/n2ns;
            b2grad = sum(e2,2)/n2ns;
            b1grad = sum(e1,2)/n2ns;

            cst  = 0.5*sum(sum(d2.^2,2))/n2ns;
            grad = [W1grad(:) ; W2grad(:) ; b1grad(:) ; b2grad(:)];
            
        end
                
        function theta = GetTheta(obj)
            
            theta = [obj.W1(:) ; obj.W2(:) ; obj.b1(:) ; obj.b2(:)];
            
        end
        
        function SetTheta(obj,theta)
           
            N0  = obj.N0;
            N1  = obj.N1;
            N2  = obj.N2;
            
            obj.W1 = reshape(theta(1:(N0*N1)), N1, N0);
            obj.W2 = reshape(theta((N0*N1+1):(N0*N1+N1*N2)), N2, N1);
            obj.b1 = theta((N0*N1+N1*N2+1):(N0*N1+N1*N2+N1));
            obj.b2 = theta((N0*N1+N1*N2+N1+1):(N0*N1+N1*N2+N1+N2));
            
        end
        
        function r2 = FProp(obj,r0)
           
            x1 = obj.W1*r0 + obj.b1;
            r1 = tanh(x1);
            x2 = obj.W2*r1 + obj.b2;
            r2 = x2;
            
            obj.r0 = r0;
            obj.r1 = r1;
            obj.r2 = r2;            
            
        end
        
        function d0 = BProp(obj,d2)
                        
            f2 = 1;
            e2 = d2.*f2;
            d1 = obj.W2'*e2;
            f1 = 1-obj.r1.^2;
            e1 = d1.*f1;
            d0 = obj.W1'*e1;
            
        end
        
    end
    
end