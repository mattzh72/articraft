from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_bottle_shell_mesh():
    outer_profile = [
        (0.0310, 0.0000),
        (0.0355, 0.0050),
        (0.0385, 0.0180),
        (0.0390, 0.0700),
        (0.0370, 0.1250),
        (0.0385, 0.1650),
        (0.0360, 0.1850),
        (0.0300, 0.1980),
        (0.0240, 0.2100),
        (0.0180, 0.2220),
        (0.0150, 0.2300),
        (0.0135, 0.2370),
    ]
    inner_profile = [
        (0.0000, 0.0040),
        (0.0280, 0.0060),
        (0.0360, 0.0180),
        (0.0360, 0.0700),
        (0.0340, 0.1250),
        (0.0350, 0.1650),
        (0.0325, 0.1840),
        (0.0265, 0.1960),
        (0.0210, 0.2070),
        (0.0160, 0.2190),
        (0.0130, 0.2280),
        (0.0108, 0.2330),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    return shell


def _build_cap_mesh():
    outer_profile = [
        (0.0206, 0.0000),
        (0.0256, 0.0013),
        (0.0256, 0.0125),
        (0.0240, 0.0145),
        (0.0240, 0.0206),
        (0.0224, 0.0222),
    ]
    inner_profile = [
        (0.0206, 0.0000),
        (0.0201, 0.0013),
        (0.0201, 0.0189),
        (0.0178, 0.0208),
        (0.0000, 0.0222),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.84, 0.93, 0.98, 0.38))
    cap_blue = model.material("cap_blue", rgba=(0.14, 0.36, 0.78, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        _save_mesh("bottle_shell", _build_bottle_shell_mesh()),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.0162, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.2190)),
        material=bottle_plastic,
        name="thread_low",
    )
    bottle.visual(
        Cylinder(radius=0.0162, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.2240)),
        material=bottle_plastic,
        name="thread_mid",
    )
    bottle.visual(
        Cylinder(radius=0.0162, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.2290)),
        material=bottle_plastic,
        name="thread_high",
    )
    bottle.visual(
        Cylinder(radius=0.0132, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.2350)),
        material=bottle_plastic,
        name="lip_ring",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0395, length=0.2370),
        mass=0.060,
        origin=Origin(xyz=(0.0, 0.0, 0.1185)),
    )

    cap = model.part("cap")
    cap_outer_radius = 0.0256
    cap_height = 0.0222
    cap.visual(
        _save_mesh("cap_shell", _build_cap_mesh()),
        material=cap_blue,
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.0106, length=0.0078),
        origin=Origin(xyz=(0.0, 0.0, 0.0183)),
        material=cap_blue,
        name="seal_plug",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=cap_outer_radius, length=cap_height),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, cap_height * 0.5)),
    )

    model.articulation(
        "bottle_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.2184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=14.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("bottle_to_cap")

    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="thread_low",
        outer_elem="cap_shell",
        margin=0.0015,
        name="cap shell stays centered over the lower bottle thread",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="thread_high",
        outer_elem="cap_shell",
        margin=0.0015,
        name="cap shell also envelopes the upper bottle thread",
    )
    ctx.expect_overlap(
        bottle,
        cap,
        axes="z",
        elem_a="thread_mid",
        elem_b="cap_shell",
        min_overlap=0.0010,
        name="cap shell spans the bottle's main thread band",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: 1.9}):
        ctx.expect_within(
            bottle,
            cap,
            axes="xy",
            inner_elem="thread_mid",
            outer_elem="cap_shell",
            margin=0.0025,
            name="spun cap remains coaxial with the bottle neck",
        )
        spun_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap rotates in place about the neck axis",
        rest_pos is not None
        and spun_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, spun_pos)) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
