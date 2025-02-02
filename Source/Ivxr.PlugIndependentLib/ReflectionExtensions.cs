﻿using System;
using System.Reflection;

namespace Iv4xr.PluginLib
{
    public static class ReflectionExtensions
    {
        public static MethodInfo GetMethodOrParentMethod(this object instance, string methodName, Type type,
            BindingFlags bindingAttr = BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)
        {
            var method = type.GetMethod(methodName, bindingAttr);
            if (method != null)
            {
                return method;
            }

            return type == typeof(object) ? null : instance.GetMethodOrParentMethod(methodName, type.GetTypeInfo().BaseType, bindingAttr);
        }

        public static TReturnType CallMethod<TReturnType>(this object instance, string methodName, object[] args)
        {
            instance.ThrowIfNull(null, "Instance to call method on is null.");
            var method = instance.GetMethodOrParentMethod(methodName, instance.GetType());
            method.ThrowIfNull(methodName, $"Method {methodName} is not found on object {instance.GetType().Name}");
            return (TReturnType)method.Invoke(instance, args);
        }

        public static TReturnType CallMethod<TReturnType>(this object instance, string methodName)
        {
            instance.ThrowIfNull(null, "Instance to call method on is null.");
            var method = instance.GetMethodOrParentMethod(methodName, instance.GetType());
            method.ThrowIfNull(methodName, $"Method {methodName} is not found on object {instance.GetType().Name}");
            return (TReturnType)method.Invoke(instance, new object[] { });
        }

        public static void SetInstanceField(this object instance, string fieldName, object value)
        {
            BindingFlags bindFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
                                     | BindingFlags.Static;
            var t = instance.GetType();
            FieldInfo field = t.GetField(fieldName, bindFlags);
            field.SetValue(instance, value);
        }

        public static void SetInstanceProperty(this object instance, string fieldName, object value)
        {
            BindingFlags bindFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
                                     | BindingFlags.Static;
            var t = instance.GetType();
            PropertyInfo field = t.GetProperty(fieldName, bindFlags);
            field.SetValue(instance, value);
        }

        public static T GetInstanceProperty<T>(this object instance, string fieldName)
        {
            BindingFlags bindFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
                                     | BindingFlags.Static;
            var t = instance.GetType();
            var field = t.GetProperty(fieldName, bindFlags);
            field.ThrowIfNull(fieldName, $"Field {fieldName} not found for type {t.Name}");
            return (T)field.GetValue(instance);
        }


        public static T GetInstanceField<T>(this object instance, string fieldName)
        {
            BindingFlags bindFlags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
                                     | BindingFlags.Static;
            var t = instance.GetType();
            var field = t.GetField(fieldName, bindFlags);
            field.ThrowIfNull(fieldName, $"Field {fieldName} not found for type {t.Name}");
            return (T)field.GetValue(instance);
        }

        public static T GetInstanceFieldOrThrow<T>(this object instance, string fieldName)
        {
            instance.ThrowIfNull(null, $"Instance of type {instance.GetType()} to get the field from is null.");
            var field = instance.GetInstanceField<T>(fieldName);
            field.ThrowIfNull(fieldName, $"Field {fieldName} of type {typeof(T)} is null!");
            return field;
        }
    }
}
