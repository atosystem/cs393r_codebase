import pandas as pd
import torch
import numpy as np
from torch import nn
from torch import optim
import argparse


def DTW_align(input_a,input_b,cost_fn=None):
    if cost_fn is None:
        cost_fn = lambda x,y: torch.norm(torch.abs(x-y))
    
    cost_table = torch.ones((input_a.size(0),input_b.size(0))) * float('inf')
    back_track_table = np.ones((input_a.size(0),input_b.size(0)),dtype=int) * -1 
    back_track_table = back_track_table
    # -1: error
    # 0: from left
    # 1: from top
    # 2: from top-left

    cost_table[0,0] = cost_fn(input_a[0],input_b[0])
    back_track_table[0,0] = -1

    for i_a in range(1,input_a.size(0)):
        cost_table[i_a,0] = cost_fn(input_a[i_a],input_b[0]) + cost_table[i_a-1,0]
        back_track_table[i_a,0] = 0
    for i_b in range(1,input_b.size(0)):
        cost_table[0,i_b] = cost_fn(input_a[0],input_b[i_b]) + cost_table[0,i_b-1]
        back_track_table[0,i_b] = 1
    
    for i_a in range(1,input_a.size(0)):
        for i_b in range(1,input_b.size(0)):
            current_cost = cost_fn(input_a[i_a],input_b[i_b])
            candidate_costs = [cost_table[i_a-1,i_b].detach(),cost_table[i_a,i_b-1].detach(),cost_table[i_a-1,i_b-1].detach()]
            back_track_table[i_a,i_b] = np.argmin(candidate_costs)
            cost_table[i_a,i_b] = current_cost + candidate_costs[np.argmin(candidate_costs)]


    # get path
    cur_node = [input_a.size(0)-1,input_b.size(0)-1]
    paths = [cur_node]

    while( 1  ):
        if paths[-1][0] == 0 and paths[-1][1] == 0:
            break
        new_back_track = back_track_table[ paths[-1][0],paths[-1][1]]
        # print("paths[-1]",paths[-1],new_back_track)
        prev_node = [0,0]
        if new_back_track == 0:
            prev_node[0] = paths[-1][0] - 1
            prev_node[1] = paths[-1][1]
        elif new_back_track == 1:
            prev_node[0] = paths[-1][0]
            prev_node[1] = paths[-1][1] - 1
        elif new_back_track == 2:
            prev_node[0] = paths[-1][0] - 1
            prev_node[1] = paths[-1][1] - 1
        else:
            raise NotImplementedError()
        
        paths.append(prev_node)
    
    # print(len(paths))

    paths.reverse()

    return cost_table[input_a.size(0)-1,input_b.size(0)-1], paths


def cal_loss(ref_poses,est_poses):

    def my_cost(x,y):
        assert x.dim() == 1
        assert y.dim() == 1
        loss = 0
        loss += torch.norm(torch.abs(x[:2]-y[:2]))
        loss += 1 - torch.cos(x[2] - y[2])
        # loss += torch.norm(torch.abs(x[2] % (2 * torch.pi)-y[2]))
        return loss
        # return torch.norm(torch.abs(x-y))

    total_cost, paths = DTW_align(ref_poses,est_poses)
    # find lowest aligned ref point
    ref_poses_aligned_ids = []
    for i_est in range(est_poses.size(0)):
        all_nodes = [x for x in paths if x[1] == i_est]
        all_nodes_cost = torch.tensor([ my_cost(ref_poses[x[0]],est_poses[i_est]) for  x in all_nodes ])
        ref_poses_aligned_ids.append(all_nodes[torch.argmin(all_nodes_cost)][0])

    loss = 0
    for i in range(len(ref_poses_aligned_ids)):
        loss+= my_cost(ref_poses[ref_poses_aligned_ids[i]],est_poses[i])
    return loss / len(ref_poses_aligned_ids),total_cost


    # print(ref_poses_aligned_ids)

# loss = cal_loss(ref_poses=ref_poses,est_poses=est_poses)


class learnedTransform(nn.Module):
    def __init__(self, *args, **kwargs):
        super(learnedTransform, self).__init__()
        self.translation = torch.nn.Parameter(torch.zeros(2))
        self.rotation = torch.nn.Parameter(torch.zeros(1))

    def forward(self, x):
        x[:,:2] = x[:,:2] + self.translation.reshape(1,2)
        # x[:,2] = torch.clamp(x[:,2] + self.rotation , -torch.pi / 2, torch.pi / 2)
        # x[:,2] = x[:,2] + torch.clamp(self.rotation,0,2* torch.pi)
        x[:,2] = x[:,2] + torch.clamp(self.rotation,-torch.pi / 2, torch.pi / 2)
        # x[:,2] = torch.clamp(x[:,2] + self.rotation , -torch.pi / 2, torch.pi / 2)
        return x



def main(args):
    reference_csv_fp = args.ref
    estimated_csv_fp = args.est
    df_ref = pd.read_csv(reference_csv_fp,index_col=None)
    df_est = pd.read_csv(estimated_csv_fp,index_col=None)


    # Step 3: Choose optimizer
    model = learnedTransform()
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    # optimizer = optim.SGD(model.parameters(), lr=0.01)

    # Step 4: Iterate over dataset
    # Example data
    # inputs = torch.tensor([[1.0], [2.0], [3.0]])
    # targets = torch.tensor([[2.0], [4.0], [6.0]])

    # Training loop
    num_epochs = args.epoch
    for epoch in range(num_epochs):
        # Forward pass
        ref_poses = torch.FloatTensor(df_ref.values)
        est_poses = torch.FloatTensor(df_est.values)

        outputs = model(est_poses)
        
        # Step 5: Compute loss
        loss, dtw_total_cost = cal_loss(ref_poses=ref_poses,est_poses=outputs)
        
        # Zero gradients, backward pass, and optimize
        optimizer.zero_grad()  # Clear gradients
        loss.backward()        # Backpropagation
        optimizer.step()       # Update parameters

        
        if (epoch+1) % 10 == 0:
            print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')


    # After training
    print("Model parameters after training:")
    for name, param in model.named_parameters():
        print(name, param.data)







if __name__ == "__main__":
    torch.manual_seed(0)
    # torch.cuda.manual_seed(0)
    # torch.backends.cudnn.deterministic = True
    # torch.backends.cudnn.benchmark = False

    parser = argparse.ArgumentParser(description="Find best pose transform between two trajectory")

    # Add arguments
    parser.add_argument("--ref", type=str,  help="reference pose csv file path")
    parser.add_argument("--est", type=str, help="estimated pose csv file path")
    parser.add_argument("--lr", type=float,  default=0.1, help="learning rate")
    parser.add_argument("--epoch", type=int, default=100 , help="numer of iteration")

    # Parse arguments from command line
    args = parser.parse_args()

    main(args)


    