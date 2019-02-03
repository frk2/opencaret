import torch
import numpy as np
import torch.nn as nn
import torch.optim as optim
import sys
from sklearn.preprocessing import normalize
import tqdm

class FFNet(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(FFNet, self).__init__()

        self.fc0 = nn.Linear(input_dim, 100)
        self.relu0 = nn.ReLU()
        # Linear function 1: 784 --> 100
        self.bn1 = nn.BatchNorm1d(num_features=100)
        self.fc1 = nn.Linear(100, 200)
        # Non-linearity 1
        self.relu1 = nn.ReLU()

        # Linear function 2: 100 --> 100
        self.bn2 = nn.BatchNorm1d(num_features=200)
        self.fc2 = nn.Linear(200, 200)
        # Non-linearity 2
        self.relu2 = nn.ReLU()

        # Linear function 3: 100 --> 64
        self.bn3 = nn.BatchNorm1d(num_features=200)

        self.fc3 = nn.Linear(200, 100)
        # Non-linearity 3
        self.relu3 = nn.ReLU()

        # Linear function 4 (readout): 64 --> 10
        self.fc4 = nn.Linear(100, output_dim)
        self.dropout = nn.Dropout2d()

    def forward(self, x):
        out = self.fc0(x)
        # Non-linearity 1
        out = self.relu0(self.bn1(out))
        out = self.dropout(out)

        # Linear function 1
        out = self.fc1(out)
        # Non-linearity 1
        out = self.relu1(self.bn2(out))
        out = self.dropout(out)

        # Linear function 2
        out = self.fc2(out)
        # Non-linearity 2
        out = self.relu2(self.bn3(out))
        out = self.dropout(out)


        # Linear function 2
        out = self.fc3(out)
        # Non-linearity 2
        out = self.relu3(out)
        out = self.dropout(out)

        # Linear function 4 (readout)
        out = self.fc4(out)
        return out

class Reader:
    def __init__(self):
        self.orig_data = np.genfromtxt(sys.argv[1], delimiter=',')
        self.orig_data = self.orig_data[2:, 1:]
        self.mean = np.mean(self.orig_data, axis=0)
        self.std = np.std(self.orig_data, axis=0)
        self.data = (self.orig_data - self.mean) / self.std
        self.cur_index = 0
        self.read_items = 9000

    def transformX(self, X):
        return ( X - self.mean[1:]) / self.std[1:]

    def transformY(self, Y):
        return (Y * self.std[0]) + self.mean[0]

    def read_tensor(self):
        items_to_read = self.read_items
        if self.data.shape[0] < self.cur_index + items_to_read:
            items_to_read = self.data.shape[0] - self.cur_index
        Y = torch.from_numpy(self.data[self.cur_index:self.cur_index + items_to_read, 0][:, np.newaxis]).cuda()
        X = torch.from_numpy(self.data[self.cur_index:self.cur_index + items_to_read, 1:]).cuda()
        self.cur_index += items_to_read
        return X, Y

    def has_more(self):
        return self.cur_index < self.data.shape[0]

def train():
    net = FFNet(3,1)
    net.cuda().double().train()
    optimizer = optim.Adam(net.parameters(), lr=0.001)
    criterion = nn.MSELoss()
    # in your training loop:
    iter = 0
    for i in tqdm.tqdm(range(2000)):
        reader = Reader()
        while reader.has_more():
            iter += 1
            optimizer.zero_grad()  # zero the gradient buffers
            input, target = reader.read_tensor()
            output = net(input)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()  # Does the update
            print(loss)

    # test = torch.Tensor([reader.transformX([-1.72635881656,2.89375589981,-1.7521002087])]).double().cuda()
    # print(reader.transformY(net(test)))
    torch.save(net, 'out-model')


if __name__ == '__main__':
    # train()
    net=torch.load('out-model')
    net.eval()
    reader = Reader()
    test = torch.Tensor([reader.transformX([0.0124583113589,-2.11359372417,0.0214688891689]),
                         reader.transformX([1.19267122845,-8.15243293607,3.93679599616]),
                         reader.transformX([2, 0.493928178314, 5.01171643427])]).double().cuda()
    print(test)
    print(reader.transformY(net(test)))
