#include "DebugRender.h"
#include "stringUtils.h"

std::string DebugRender::commitImpl()
{
    static const std::string sphereTemplate = R"(
    {
      "Sphere": {
        "x": %x,
        "y": %y,
        "z": %z,
        "radius": %rad,
        "r": %red,
        "g": %green,
        "b": %blue,
        "a": %alpha
      }
    })";

    static const std::string textTemplate = R"(
    {
      "Text": "%string"
    })";

    static const std::string packetTemplate = R"(
[
    %spheres
    %texts
])";

    std::string spheres;
    spheres.reserve(m_spheres.size() * sphereTemplate.size() * 2);

    for(const Sphere& sphere : m_spheres)
    {
        FormattedString next { sphereTemplate };

        next.format("%x", sphere.center.x)
            .format("%y", sphere.center.y)
            .format("%z", sphere.center.z)

            .format("%rad", sphere.radius)

            .format("%red",   sphere.color[0])
            .format("%green", sphere.color[1])
            .format("%blue",  sphere.color[2])
            .format("%alpha", sphere.color[3]);

        spheres += spheres.empty() ? "" : ",";
        spheres += next.get();
    }

    std::string texts;
    texts.reserve(m_strings.size() * 128);
    spheres += (spheres.empty() || m_strings.empty()) ? "" : ",";
    for(const std::string& s : m_strings)
    {
        std::string next = textTemplate;
        texts   += texts.empty() ? "" : ",";
        texts   += format(next, "%string", s);
    }

    FormattedString packet = packetTemplate;
    packet.format("%spheres", spheres)
          .format("%texts",   texts);

    m_spheres.clear();
    m_strings.clear();
    m_isEnabled = false;
    return packet.get();
}
