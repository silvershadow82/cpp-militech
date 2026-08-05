#include "mission_control/explorer.hpp"

namespace mission_control {

void Explorer::observe(const Observation& observation)
{
  this->memory.update(observation);
  this->robot = observation.robot;
  this->observed = true;
  this->visibleContacts.clear();

  for (const auto& observed : observation.cells) {
    if (observed.state == CellState::Contact) {
      this->visibleContacts.push_back(VisibleContact{observed.contactId, observed.cell});
    }
    else if (observed.state == CellState::Processed) {
      this->engaged.erase(observed.contactId);
    }
  }
}

Decision Explorer::decide() const
{
  Decision decision;

  if (!this->observed) {
    return decision;
  }

  if (const auto contact = pendingContact(); contact.has_value()) {
    decision.kind = Decision::Kind::Engage;
    decision.contact = *contact;
    return decision;
  }

  if (!this->visibleContacts.empty()) {
    return decision;
  }

  const auto targets = this->memory.frontiers();
  if (targets.empty()) {
    decision.kind = Decision::Kind::Done;
    return decision;
  }

  const auto direction = this->memory.firstStepToward(this->robot, targets);
  if (!direction.has_value()) {
    decision.kind = Decision::Kind::Failed;
    return decision;
  }

  decision.kind = Decision::Kind::Move;
  decision.direction = *direction;
  return decision;
}

void Explorer::markEngaged(const int contact_id)
{
  this->engaged.insert(contact_id);
}

void Explorer::forgetEngaged(const int contact_id)
{
  this->engaged.erase(contact_id);
}

bool Explorer::isEngaged(const int contact_id) const
{
  return this->engaged.find(contact_id) != this->engaged.end();
}

const MapMemory& Explorer::mapMemory() const
{
  return this->memory;
}

bool Explorer::explorationComplete() const
{
  return this->observed && this->visibleContacts.empty() && this->memory.frontiers().empty();
}

std::optional<VisibleContact> Explorer::pendingContact() const
{
  std::optional<VisibleContact> pending;

  for (const auto& contact : this->visibleContacts) {
    if (isEngaged(contact.contactId)) {
      continue;
    }
    if (!pending.has_value() || contact.contactId < pending->contactId) {
      pending = contact;
    }
  }

  return pending;
}

}  // namespace mission_control
