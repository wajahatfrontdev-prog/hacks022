from typing import List
def get_embeddings(texts: List[str]) -> List[List[float]]:
    return [[0.0] * 768 for _ in texts]
