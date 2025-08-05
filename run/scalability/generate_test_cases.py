# source: https://github.com/RoboChor/robethichor-ethics-based-negotiation
import json
import random
import os
import argparse

def generate_ethical_implications(n):
    implications = {
        "t1": [f'd{i}' for i in range(1, n + 1)]
    }
    with open(f"test_cases/ethical_implications.json", "w") as json_file:
        json.dump(implications, json_file, indent=4)

def generate_disposition_activatons(n):
    dispositions = {f"s{i}": [f"d{i}"] for i in range(1, n + 1)}
    if not os.path.exists(f"test_cases/"):
        os.makedirs(f"test_cases/")
    with open(f"test_cases/disposition_activation.json", "w") as json_file:
        json.dump(dispositions, json_file, indent=4)

def generate_context(n):
    if not os.path.exists(f"test_cases/"):
        os.makedirs(f"test_cases/")
    with open(f"test_cases/context.json", "w") as f:
        json.dump({"location": "sim"}, f, indent=4)

def generate_ethic_profiles(n, file_number):
    ethic_profiles = {
        "sim": {f"d{i+1}": random.randint(0, 5) for i in range(n)}
    }
    if not os.path.exists(f"test_cases/{n}/profiles/"):
        os.makedirs(f"test_cases/{n}/profiles/")
    with open(f"test_cases/{n}/profiles/ethic_profiles_{n}-{file_number}.json", "w") as f:
        json.dump(ethic_profiles, f, indent=4)

def generate_user_status(n, p, file_number):
    user_status = {f"s{i+1}": False for i in range(n)}
    true_count = int((n / 100) * p)
    true_indices = random.sample(range(n), true_count)
    for i in true_indices:
        user_status[f"s{i+1}"] = True
    if not os.path.exists(f"test_cases/{n}/statuses/{p}/"):
        os.makedirs(f"test_cases/{n}/statuses/{p}/")
    with open(f"test_cases/{n}/statuses/{p}/user_status_{n}_{p}-{file_number}.json", "w") as f:
        json.dump(user_status, f, indent=4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ethic profiles and user status.")
    parser.add_argument('--n', nargs='+', type=int, required=True, help="List of number of dispositions to generate")
    parser.add_argument('--p', nargs='+', type=int, required=True, help="List of percentages of activated dispositions")
    parser.add_argument('--c', type=int, required=True, help="Number of test cases to generate for each configuration")

    args = parser.parse_args()

    N_values = args.n
    P_values = args.p
    max_i = args.c

    for n in N_values:
        print(f"Experiment size {n}, generating configuration files...")
        generate_context(n)
        generate_ethical_implications(n)
        generate_disposition_activatons(n)
        for i in range(1, max_i + 1):
            print(f"UC #{i}:")
            print(f"Generating ethic profiles for n={n}...")
            generate_ethic_profiles(n, i)
            for p in P_values:
                print(f"Generating user status for p={p}...")
                generate_user_status(n, p, i)