using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AStar
{
	public static Node[] FindPath(Vector2 startPos, Vector2 targetPos)
	{
		Node startNode = GridManager.Instance.NodeFromWorldPoint(startPos);
		Node targetNode = GridManager.Instance.NodeFromWorldPoint(targetPos);

		if (startNode == null || targetNode == null || startNode == targetNode) {
			return null;
		}

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				Node[] path = RetracePath(startNode, targetNode).ToArray();
				path[path.Length - 1].worldPosition = targetPos;
				return path;
			}

			foreach (Node neighbour in GridManager.Instance.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		return null;
	}

	static List<Node> RetracePath(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();

		return path;

	}

	static int GetDistance(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14 * dstY + 10 * (dstX - dstY);
		return 14 * dstX + 10 * (dstY - dstX);
	}

	public static float PathLength(Node[] path) {

		if (path == null) {
			return 0;
		}

		float length = 0;
		for (int i = 1; i < path.Length; i++) {
			length += Vector2.Distance(path[i-1].worldPosition, path[i].worldPosition);
		}

		return length;
	}
}
