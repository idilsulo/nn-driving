"""
model.py
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()

        self.conv1 = nn.Conv2d(3, 32, kernel_size=4, stride=2, padding=1)
        self.bn1 = nn.BatchNorm2d(32)
        self.l_relu1 = nn.LeakyReLU(0.2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1)
        self.bn2 = nn.BatchNorm2d(64)
        self.l_relu2 = nn.LeakyReLU(0.2)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1)
        self.bn3 = nn.BatchNorm2d(128)
        self.l_relu3 = nn.LeakyReLU(0.2)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=4, stride=2, padding=1)
        self.bn4 = nn.BatchNorm2d(256)
        self.l_relu4 = nn.LeakyReLU(0.2)
        self.conv5 = nn.Conv2d(256, 512, kernel_size=4, stride=2, padding=1)
        self.bn5 = nn.BatchNorm2d(512)
        self.l_relu5 = nn.LeakyReLU(0.2)
        self.conv6 = nn.Conv2d(512, 64, kernel_size=4, stride=1, padding=0)

        self.fc1 = nn.Linear(64, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 25)
        self.fc4 = nn.Linear(25, 15)
        self.fc5 = nn.Linear(15, 8)
        self.fc6 = nn.Linear(8, 1)


    def forward(self, x):
        x = self.l_relu1(self.bn1(self.conv1(x)))
        x = self.l_relu2(self.bn2(self.conv2(x)))
        x = self.l_relu3(self.bn3(self.conv3(x)))
        x = self.l_relu4(self.bn4(self.conv4(x)))
        x = self.l_relu5(self.bn5(self.conv5(x)))
        x = self.conv6(x)
        
        x = x.view(-1, 64)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        x = self.fc6(x)
        return x


if __name__ == '__main__':

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    model = CNN().to(device)
    print("Printing model")
    print(model)

    from torchsummary import summary
    print("Printing summary")
    summary(model, (3, 128, 128))
