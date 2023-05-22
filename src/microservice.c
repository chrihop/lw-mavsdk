#include "lwmavsdk.h"

static void lwm_service_pool_init(struct lwm_service_pool_t * pool)
{
    pool->n = 0;
    for (uint32_t i = 0; i < MAX_LWM_SERVICE; i++)
    {
        pool->services[i].is_active = false;
    }
}

static struct lwm_microservice_t * lwm_service_pool_alloc(struct lwm_service_pool_t * pool)
{
    if (pool->n >= MAX_LWM_SERVICE)
    {
        return NULL;
    }
    for (uint32_t i = 0; i < MAX_LWM_SERVICE; i++)
    {
        if (!pool->services[i].is_active)
        {
            pool->services[i].is_active = true;
            pool->n++;
            return &pool->services[i];
        }
    }
    return NULL;
}

static void lwm_service_pool_free(struct lwm_service_pool_t * pool, struct lwm_microservice_t * service)
{
    service->is_active = false;
    pool->n--;
}


static void lwm_service_list_init(struct lwm_service_list_t * list)
{
    list->head = NULL;
    list->tail = NULL;
}

static void lwm_service_push_tail(struct lwm_service_list_t * list, struct lwm_microservice_t * service)
{
    if (list->head == NULL)
    {
        list->head = service;
        list->tail = service;
    }
    else
    {
        list->tail->next = service;
        service->prev = list->tail;
        list->tail = service;
    }
}

static void lwm_service_remove(struct lwm_service_list_t * list, struct lwm_microservice_t * service)
{
    if (service->prev == NULL)
    {
        list->head = service->next;
    }
    else
    {
        service->prev->next = service->next;
    }

    if (service->next == NULL)
    {
        list->tail = service->prev;
    }
    else
    {
        service->next->prev = service->prev;
    }
}

static struct lwm_microservice_t * lwm_service_head(struct lwm_service_list_t * list)
{
    return list->head;
}

static struct lwm_microservice_t * lwm_service_next(struct lwm_microservice_t * service)
{
    return service->next;
}

static bool lwm_service_on_list(struct lwm_service_list_t * list, struct lwm_microservice_t * service)
{
    struct lwm_microservice_t * s = lwm_service_head(list);
    while (s != NULL)
    {
        if (s == service)
        {
            return true;
        }
        s = lwm_service_next(s);
    }
    return false;
}

static void lwm_service_foreach(struct lwm_service_list_t * list, mavlink_message_t * msg)
{
    struct lwm_microservice_t * service = lwm_service_head(list);
    while (service != NULL)
    {
        service->handler(service->context, msg);
        service = lwm_service_next(service);
    }
}

static void lwm_microservice_registry_init(struct lwm_microservice_registry_t * registry)
{
    registry->n = 0;
    for (uint32_t i = 0; i < MAX_LWM_SERVICE_REGISTRY; i++)
    {
        registry->entries[i].is_active = false;
    }
}

static struct lwm_microservice_registry_entry_t * lwm_microservice_registry_alloc(struct lwm_microservice_registry_t * registry)
{
    if (registry->n >= MAX_LWM_SERVICE_REGISTRY)
    {
        return NULL;
    }
    for (uint32_t i = 0; i < MAX_LWM_SERVICE_REGISTRY; i++)
    {
        if (!registry->entries[i].is_active)
        {
            registry->entries[i].is_active = true;
            registry->n++;
            return &registry->entries[i];
        }
    }
    return NULL;
}

static void lwm_microservice_registry_free(struct lwm_microservice_registry_t * registry, struct lwm_microservice_registry_entry_t * entry)
{
    entry->is_active = false;
    registry->n--;
}

static struct lwm_microservice_registry_entry_t * lwm_microservice_registry_find(struct lwm_microservice_registry_t * registry, uint32_t msgid)
{
    for (uint32_t i = 0; i < MAX_LWM_SERVICE_REGISTRY; i++)
    {
        if (registry->entries[i].is_active && registry->entries[i].msgid == msgid)
        {
            return &registry->entries[i];
        }
    }
    return NULL;
}

static struct lwm_microservice_registry_entry_t * lwm_microservice_registry_find_or_alloc(struct lwm_microservice_registry_t * registry, uint32_t msgid)
{
    struct lwm_microservice_registry_entry_t * entry = lwm_microservice_registry_find(registry, msgid);
    if (entry == NULL)
    {
        entry = lwm_microservice_registry_alloc(registry);
        if (entry != NULL)
        {
            entry->msgid = msgid;
            lwm_service_list_init(&entry->list);
        }
    }
    return entry;
}

static void lwm_microservice_registry_add(struct lwm_microservice_registry_t * registry, uint32_t msgid, struct lwm_microservice_t * service)
{
    struct lwm_microservice_registry_entry_t * entry = lwm_microservice_registry_find_or_alloc(registry, msgid);
    if (entry != NULL && !lwm_service_on_list(&entry->list, service))
    {
        lwm_service_push_tail(&entry->list, service);
    }
}

static void lwm_microservice_registry_remove(struct lwm_microservice_registry_t * registry, uint32_t msgid, struct lwm_microservice_t * service)
{
    struct lwm_microservice_registry_entry_t * entry = lwm_microservice_registry_find(registry, msgid);
    if (entry != NULL)
    {
        lwm_service_remove(&entry->list, service);
    }
}

static void lwm_microservice_registry_remove_all(struct lwm_microservice_registry_t * registry, struct lwm_microservice_t * service)
{
    for (uint32_t i = 0; i < MAX_LWM_SERVICE_REGISTRY; i++)
    {
        struct lwm_microservice_registry_entry_t * entry = &registry->entries[i];
        if (entry->is_active && lwm_service_on_list(&entry->list, service))
        {
            lwm_service_remove(&entry->list, service);
        }
    }
}

void lwm_microservice_init(struct lwm_vehicle_t * vehicle)
{
    lwm_microservice_registry_init(&vehicle->registry);
    lwm_service_pool_init(&vehicle->service_pool);
}

void lwm_microservice_process(struct lwm_vehicle_t * vehicle, mavlink_message_t * msg)
{
    struct lwm_microservice_registry_entry_t * entry = lwm_microservice_registry_find(&vehicle->registry, msg->msgid);
    if (entry != NULL)
    {
        lwm_service_foreach(&entry->list, msg);
    }
}

enum lwm_error_t lwm_microservice_add_to(struct lwm_vehicle_t * vehicle, uint32_t msgid, struct lwm_microservice_t * service)
{
    if (service->handler == NULL)
    {
        return LWM_ERR_BAD_PARAM;
    }
    lwm_microservice_registry_add(&vehicle->registry, msgid, service);
    return LWM_OK;
}

enum lwm_error_t lwm_microservice_remove_from(struct lwm_vehicle_t * vehicle, uint32_t msgid, struct lwm_microservice_t * service)
{
    lwm_microservice_registry_remove(&vehicle->registry, msgid, service);
    return LWM_OK;
}

struct lwm_microservice_t * lwm_microservice_create(struct lwm_vehicle_t * vehicle)
{
    return lwm_service_pool_alloc(&vehicle->service_pool);
}

void lwm_microservice_destroy(struct lwm_vehicle_t * vehicle, struct lwm_microservice_t * service)
{
    lwm_microservice_registry_remove_all(&vehicle->registry, service);
    lwm_service_pool_free(&vehicle->service_pool, service);
}
