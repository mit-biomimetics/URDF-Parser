/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen */

#ifndef DYNACORE_URDF_INTERFACE_MODEL_H
#define DYNACORE_URDF_INTERFACE_MODEL_H

#include <string>
#include <map>
#include <stack>
#include <algorithm>
// #include "link.h"
// #include "constraint_joint.h"
#include "cluster.h"
#include "exception.h"
#include <memory>

namespace urdf
{

  class ModelInterface
  {
  public:
    std::shared_ptr<const Link> getRoot(void) const { return this->root_link_; };
    std::shared_ptr<const Link> getLink(const std::string &name) const
    {
      std::shared_ptr<const Link> ptr;
      if (this->links_.find(name) == this->links_.end())
        ptr.reset();
      else
        ptr = this->links_.find(name)->second;
      return ptr;
    };

    std::shared_ptr<const Joint> getJoint(const std::string &name) const
    {
      std::shared_ptr<const Joint> ptr;
      if (this->joints_.find(name) == this->joints_.end())
        ptr.reset();
      else
        ptr = this->joints_.find(name)->second;
      return ptr;
    };

    std::shared_ptr<const ConstraintJoint> getConstraint(const std::string &name) const
    {
      std::shared_ptr<const ConstraintJoint> ptr;
      if (this->constraint_joints_.find(name) == this->constraint_joints_.end())
        ptr.reset();
      else
        ptr = this->constraint_joints_.find(name)->second;
      return ptr;
    };

    // TODO(@MatthewChignoli): Cleaner accessor
    const std::map<std::string, std::shared_ptr<Cluster>> &getClusters() const { return clusters_; }

    const std::string &getName() const { return name_; };
    void getLinks(std::vector<std::shared_ptr<Link>> &links) const
    {
      for (std::map<std::string, std::shared_ptr<Link>>::const_iterator link = this->links_.begin(); link != this->links_.end(); link++)
      {
        links.push_back(link->second);
      }
    };

    // TODO(@MatthewChignoli): Proximal links are at the back of the chain, should we reverse this? Probably yes...
    void getSupportingChain(const std::string &link_name,
                            std::vector<std::shared_ptr<Link>> &supporting_chain) const
    {
      supporting_chain.clear();

      std::shared_ptr<Link> link;
      this->getLink(link_name, link);
      while (link)
      {
        supporting_chain.push_back(link);
        link = link->getParent();
      }
    }

    // Note: the src link is NEVER included in the subtree but the dst link typically is
    void getSubtreeBetweenLinks(const std::string &src_name,
                                const std::string &dst_name,
                                std::vector<std::shared_ptr<Link>> &subtree) const
    {
      subtree.clear();

      std::shared_ptr<Link> src_link, dst_link;
      this->getLink(src_name, src_link);
      this->getLink(dst_name, dst_link);

      if (!src_link)
      {
        throw ParseError("Link [" + src_name + "] not found");
      }
      if (!dst_link)
      {
        throw ParseError("Link [" + dst_name + "] not found");
      }

      // If the links are the same, return an empty subtree
      if (src_link == dst_link)
      {
        return;
      }

      // Get the supporting chain for the src and dst links
      std::vector<std::shared_ptr<Link>> dst_supporting_chain;
      getSupportingChain(dst_link->name, dst_supporting_chain);

      // Loop through the destination supporting chain until we hit the source link
      std::vector<std::shared_ptr<Link>>::reverse_iterator dst_it = dst_supporting_chain.rbegin();
      while (dst_it != dst_supporting_chain.rend() && *dst_it != src_link)
      {
        dst_it++;
      }

      // Throw error if we didn't find the source link
      if (dst_it == dst_supporting_chain.rend())
      {
        throw ParseError("Link [" + src_name + "] does not support link [" + dst_name + "]");
      }

      // Add links to subtree until reaching the destination link
      dst_it++; // Increment so that the source link is not included in the subtree
      for (; dst_it != dst_supporting_chain.rend(); dst_it++)
      {
        subtree.push_back(*dst_it);
      }
    }

    void clear()
    {
      name_.clear();
      this->links_.clear();
      this->joints_.clear();
      this->materials_.clear();
      this->root_link_.reset();
    };

    /// non-const getLink()
    void getLink(const std::string &name, std::shared_ptr<Link> &link) const
    {
      std::shared_ptr<Link> ptr;
      if (this->links_.find(name) == this->links_.end())
        ptr.reset();
      else
        ptr = this->links_.find(name)->second;
      link = ptr;
    };

    // non-const getConstraint()
    void getConstraint(const std::string &name, std::shared_ptr<ConstraintJoint> &constraint) const
    {
      std::shared_ptr<ConstraintJoint> ptr;
      if (this->constraint_joints_.find(name) == this->constraint_joints_.end())
        ptr.reset();
      else
        ptr = this->constraint_joints_.find(name)->second;
      constraint = ptr;
    };

    /// non-const getMaterial()
    std::shared_ptr<Material> getMaterial(const std::string &name) const
    {
      std::shared_ptr<Material> ptr;
      if (this->materials_.find(name) == this->materials_.end())
        ptr.reset();
      else
        ptr = this->materials_.find(name)->second;
      return ptr;
    };

    void initTree(std::map<std::string, std::string> &parent_link_tree)
    {
      // loop through all joints, for every link, assign children links and children joints
      for (std::map<std::string, std::shared_ptr<Joint>>::iterator joint = this->joints_.begin(); joint != this->joints_.end(); joint++)
      {
        std::string parent_link_name = joint->second->parent_link_name;
        std::string child_link_name = joint->second->child_link_name;

        if (parent_link_name.empty() || child_link_name.empty())
        {
          throw ParseError("Joint [" + joint->second->name + "] is missing a parent and/or child link specification.");
        }
        else
        {
          // find child and parent links
          std::shared_ptr<Link> child_link, parent_link;
          this->getLink(child_link_name, child_link);
          if (!child_link)
          {
            throw ParseError("child link [" + child_link_name + "] of joint [" + joint->first + "] not found");
          }
          this->getLink(parent_link_name, parent_link);
          if (!parent_link)
          {
            throw ParseError("parent link [" + parent_link_name + "] of joint [" + joint->first + "] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [" + joint->first + "] from your urdf file, or add \"<link name=\"" + parent_link_name + "\" />\" to your urdf file.");
          }

          // set parent link for child link
          child_link->setParent(parent_link);

          // set parent joint for child link
          child_link->parent_joint = joint->second;

          // set child joint for parent link
          parent_link->child_joints.push_back(joint->second);

          // set child link for parent link
          parent_link->child_links.push_back(child_link);

          // child links are neighbors of parent link
          parent_link->neighbors.push_back(child_link);

          // fill in child/parent string map
          parent_link_tree[child_link->name] = parent_link_name;
        }
      }

      // TODO(@MatthewChignoli): Clean this part up later. There are too many messy comments and the implementation is really inefficient
      for (auto &constraint : this->constraint_joints_)
      {
        std::string parent_link_name = constraint.second->parent_link_name;
        std::string child_link_name = constraint.second->child_link_name;

        if (parent_link_name.empty() || child_link_name.empty())
        {
          throw ParseError("ConstraintJoint [" + constraint.second->name + "] is missing a parent and/or child link specification.");
        }
        else
        {
          // find child and parent links
          std::shared_ptr<Link> child_link, parent_link;
          this->getLink(child_link_name, child_link);
          if (!child_link)
          {
            throw ParseError("child link [" + child_link_name + "] of constraint [" + constraint.first + "] not found");
          }
          this->getLink(parent_link_name, parent_link);
          if (!parent_link)
          {
            throw ParseError("parent link [" + parent_link_name + "] of constraint [" + constraint.first + "] not found.  This is not valid according to the URDF spec. Every link you refer to from a constraint needs to be explicitly defined in the robot description. To fix this problem you can either remove this constraint [" + constraint.first + "] from your urdf file, or add \"<link name=\"" + parent_link_name + "\" />\" to your urdf file.");
          }

          // Add the constraint joint to the parent link
          parent_link->constraint_joint_names.push_back(constraint.first);

          // Get supporting trees starting from nearest common ancestor and ending with the
          // parent and child links
          std::shared_ptr<Link> nca_link = nearestCommonAncestor(parent_link, child_link);
          std::vector<std::shared_ptr<Link>> nca_to_parent_subtree, nca_to_child_subtree;
          getSubtreeBetweenLinks(nca_link->name, parent_link->name, nca_to_parent_subtree);
          getSubtreeBetweenLinks(nca_link->name, child_link->name, nca_to_child_subtree);
          constraint.second->nca_to_parent_subtree = nca_to_parent_subtree;
          constraint.second->nca_to_child_subtree = nca_to_child_subtree;

          // Create cycle for the loop constraint
          parent_link->neighbors.push_back(nca_to_child_subtree.front());
          child_link->neighbors.push_back(nca_to_parent_subtree.front());
        }
      }

      // TODO(@MatthewChignoli): How can the clusters have names? I think the names need to be automatic, because the clusters are not defined in the URDF, so then the names cannot be. And we cannot just name the cluster after the constraint joint because there can be multiple constraint joints in a cluster. Maybe we can just append the name?
      // And then from there we implement an SCC algorithm to optimally cluster the bodies?
      extractStronglyConnectedComponents();

      // loop through all links, for every link, find the cluster that contains it and the clusters that contain its child links. Then assign parent and child clusters
      for (std::map<std::string, std::shared_ptr<Link>>::iterator link = this->links_.begin();
           link != this->links_.end(); link++)
      {
        std::shared_ptr<Cluster> parent_cluster = getClusterContaining(link->first);

        for (const std::string &constraint_name : link->second->constraint_joint_names)
        {
          std::shared_ptr<ConstraintJoint> constraint;
          getConstraint(constraint_name, constraint);
          parent_cluster->constraint_joints.push_back(constraint);
        }

        for (const std::shared_ptr<Link> &child_link : link->second->child_links)
        {
          std::shared_ptr<Cluster> child_cluster = getClusterContaining(child_link->name);

          // Check if child cluster is the same as parent cluster
          if (child_cluster == parent_cluster)
          {
            continue;
          }

          // Check if child cluster is already a child of the parent cluster
          if (std::find(parent_cluster->child_clusters.begin(), parent_cluster->child_clusters.end(),
                        child_cluster) != parent_cluster->child_clusters.end())
          {
            continue;
          }

          parent_cluster->child_clusters.push_back(child_cluster);
          child_cluster->setParent(parent_cluster);
        }
      }
    }

    std::shared_ptr<Cluster> getClusterContaining(const std::string &link_name)
    {
      for (std::map<std::string, std::shared_ptr<Cluster>>::iterator cluster = this->clusters_.begin(); cluster != this->clusters_.end(); cluster++)
      {
        for (const std::shared_ptr<Link> &link : cluster->second->links)
        {
          if (link->name == link_name)
          {
            return cluster->second;
          }
        }
      }
      throw ParseError("Link [" + link_name + "] not found in any cluster");
    }

    // TODO(@MatthewChignoli): Should this be a static function?
    std::shared_ptr<Link> nearestCommonAncestor(const std::shared_ptr<Link> &link1,
                                                const std::shared_ptr<Link> &link2) const
    {
      std::shared_ptr<Link> link1_ptr = link1;
      std::shared_ptr<Link> link2_ptr = link2;

      // if either link is NULL, return NULL
      if (!link1_ptr || !link2_ptr)
        return std::shared_ptr<Link>();

      // if both links are the same, return the link
      if (link1_ptr == link2_ptr)
        return link1_ptr;

      // if either link is the root, return the root
      if (link1_ptr == this->root_link_ || link2_ptr == this->root_link_)
        return this->root_link_;

      // if either link is the child of the other, return the parent
      if (link1_ptr->getParent() == link2_ptr)
        return link2_ptr;
      if (link2_ptr->getParent() == link1_ptr)
        return link1_ptr;

      // Build the supporting tree for each link, find the nearest common ancestor
      std::vector<std::shared_ptr<Link>> link1_supports, link2_supports;
      getSupportingChain(link1_ptr->name, link1_supports);
      getSupportingChain(link2_ptr->name, link2_supports);

      std::shared_ptr<Link> ancestor;
      std::vector<std::shared_ptr<Link>>::reverse_iterator link1_it = link1_supports.rbegin();
      std::vector<std::shared_ptr<Link>>::reverse_iterator link2_it = link2_supports.rbegin();
      while (link1_it != link1_supports.rend() &&
             link2_it != link2_supports.rend() &&
             *link1_it == *link2_it)
      {
        ancestor = *link1_it;
        link1_it++;
        link2_it++;
      }
      return ancestor;
    }

    // TODO(@MatthewChignoli): Maybe move these SCC functions to a separate file?
    void dfs_first_pass(const std::string &link_name,
                        std::map<std::string, bool> &visited,
                        std::stack<std::string> &finishing_order)
    {
      visited[link_name] = true;
      for (const std::shared_ptr<Link> neighbor : links_.at(link_name)->neighbors)
      {
        const std::string &neighbor_name = neighbor->name;
        if (!visited[neighbor_name])
        {
          dfs_first_pass(neighbor_name, visited, finishing_order);
        }
      }
      finishing_order.push(link_name);
    }

    void dfs_second_pass(const std::map<std::string, std::vector<std::shared_ptr<Link>>> &reverse_graph,
                         const std::string &link_name, std::map<std::string, bool> &visited,
                         std::vector<std::shared_ptr<Link>> &scc)
    {
      visited[link_name] = true;
      scc.push_back(links_.at(link_name));
      for (const std::shared_ptr<Link> neighbor : reverse_graph.at(link_name))
      {
        const std::string &neighbor_name = neighbor->name;
        if (!visited[neighbor_name])
        {
          dfs_second_pass(reverse_graph, neighbor_name, visited, scc);
        }
      }
    }

    // TODO(@MatthewChignoli): Better name?
    void extractStronglyConnectedComponents()
    {
      std::map<std::string, bool> visited;
      std::stack<std::string> finishing_order;

      // Build the reverse graph
      std::map<std::string, std::vector<std::shared_ptr<Link>>> reverse_link_graph;
      for (auto &link : this->links_)
      {
        reverse_link_graph[link.first] = std::vector<std::shared_ptr<Link>>();
      }
      for (auto &link : this->links_)
      {
        for (auto &neighbor : link.second->neighbors)
        {
          reverse_link_graph[neighbor->name].push_back(link.second);
        }
      }

      // First Pass: Calculate finishing times
      for (auto &link : this->links_)
      {
        const std::string &link_name = link.first;
        if (!visited[link_name])
        {
          dfs_first_pass(link_name, visited, finishing_order);
        }
      }

      visited.clear();

      // Second Pass: Find SCCs
      while (!finishing_order.empty())
      {
        const std::string link_name = finishing_order.top();
        finishing_order.pop();

        if (!visited[link_name])
        {
          std::vector<std::shared_ptr<Link>> scc;
          dfs_second_pass(reverse_link_graph, link_name, visited, scc);
          // TODO(@MatthewChignoli): How to name/identify clusters?
          clusters_.insert(make_pair(link_name, new Cluster()));
          clusters_.at(link_name)->name = link_name;
          clusters_.at(link_name)->links = scc;
        }
      }
    }

    void initRoot(const std::map<std::string, std::string> &parent_link_tree)
    {
      this->root_link_.reset();

      // find the links that have no parent in the tree
      for (std::map<std::string, std::shared_ptr<Link>>::const_iterator l = this->links_.begin(); l != this->links_.end(); l++)
      {
        std::map<std::string, std::string>::const_iterator parent = parent_link_tree.find(l->first);
        if (parent == parent_link_tree.end())
        {
          // store root link
          if (!this->root_link_)
          {
            getLink(l->first, this->root_link_);
          }
          // we already found a root link
          else
          {
            throw ParseError("Two root links found: [" + this->root_link_->name + "] and [" + l->first + "]");
          }
        }
      }
      if (!this->root_link_)
      {
        throw ParseError("No root link found. The robot xml is not a valid tree.");
      }
    }

    /// \brief complete list of Links
    std::map<std::string, std::shared_ptr<Link>> links_;
    /// \brief complete list of Joints
    std::map<std::string, std::shared_ptr<Joint>> joints_;
    /// \brief complete list of Constraint Joints
    std::map<std::string, std::shared_ptr<ConstraintJoint>> constraint_joints_;
    /// \brief complete list of Clusters
    // TODO(@MatthewChignoli): Should this be a map?
    std::map<std::string, std::shared_ptr<Cluster>> clusters_;
    /// \brief complete list of Materials
    std::map<std::string, std::shared_ptr<Material>> materials_;

    /// \brief The name of the robot model
    std::string name_;

    /// \brief The root is always a link (the parent of the tree describing the robot)
    std::shared_ptr<Link> root_link_;
  };

}
#endif
