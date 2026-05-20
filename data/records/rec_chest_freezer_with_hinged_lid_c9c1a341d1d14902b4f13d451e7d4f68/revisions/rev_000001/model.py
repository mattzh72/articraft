from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _open_tub_mesh(width: float, depth: float, height: float, wall: float, bottom: float) -> MeshGeometry:
    """A single connected open-top rectangular freezer cabinet shell."""
    x0, x1 = -width / 2.0, width / 2.0
    y0, y1 = -depth / 2.0, depth / 2.0
    z0, z1 = 0.0, height
    xi0, xi1 = x0 + wall, x1 - wall
    yi0, yi1 = y0 + wall, y1 - wall
    zi0 = bottom

    geom = MeshGeometry()
    vertices: dict[tuple[float, float, float], int] = {}

    def v(pt: tuple[float, float, float]) -> int:
        key = tuple(round(c, 6) for c in pt)
        if key not in vertices:
            vertices[key] = geom.add_vertex(*pt)
        return vertices[key]

    def quad(a, b, c, d) -> None:
        ia, ib, ic, id_ = v(a), v(b), v(c), v(d)
        geom.add_face(ia, ib, ic)
        geom.add_face(ia, ic, id_)

    # Exterior walls and bottom.
    quad((x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1))  # front
    quad((x1, y0, z0), (x0, y0, z0), (x0, y0, z1), (x1, y0, z1))  # rear
    quad((x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1))  # left side
    quad((x1, y1, z0), (x1, y0, z0), (x1, y0, z1), (x1, y1, z1))  # right side
    quad((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0))  # underside

    # Top rim around the open bin.
    quad((x0, y1, z1), (x1, y1, z1), (xi1, yi1, z1), (xi0, yi1, z1))
    quad((x1, y0, z1), (x0, y0, z1), (xi0, yi0, z1), (xi1, yi0, z1))
    quad((x0, y0, z1), (x0, y1, z1), (xi0, yi1, z1), (xi0, yi0, z1))
    quad((x1, y1, z1), (x1, y0, z1), (xi1, yi0, z1), (xi1, yi1, z1))

    # Interior liner faces and the bottom of the cold well.
    quad((xi1, yi1, zi0), (xi0, yi1, zi0), (xi0, yi1, z1), (xi1, yi1, z1))
    quad((xi0, yi0, zi0), (xi1, yi0, zi0), (xi1, yi0, z1), (xi0, yi0, z1))
    quad((xi0, yi1, zi0), (xi0, yi0, zi0), (xi0, yi0, z1), (xi0, yi1, z1))
    quad((xi1, yi0, zi0), (xi1, yi1, zi0), (xi1, yi1, z1), (xi1, yi0, z1))
    quad((xi0, yi0, zi0), (xi0, yi1, zi0), (xi1, yi1, zi0), (xi1, yi0, zi0))

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_residential_chest_freezer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    liner = model.material("white_inner_liner", rgba=(0.86, 0.88, 0.86, 1.0))
    gasket = model.material("dark_gray_gasket", rgba=(0.07, 0.075, 0.075, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    metal = model.material("brushed_hinge_metal", rgba=(0.62, 0.64, 0.62, 1.0))

    width = 1.18
    depth = 0.66
    cabinet_h = 0.82
    wall = 0.055
    bottom = 0.085
    front_y = depth / 2.0
    rear_y = -depth / 2.0

    lid_w = width + 0.08
    lid_rear_y = rear_y + 0.005
    lid_front_y = front_y + 0.055
    lid_depth = lid_front_y - lid_rear_y
    lid_t = 0.055
    lid_bottom = cabinet_h + 0.006
    lid_top = lid_bottom + lid_t

    hinge_y = rear_y - 0.050
    hinge_z = lid_bottom + 0.033
    hinge_xs = (-0.36, 0.36)
    hinge_w = 0.20
    knuckle_gap = 0.003
    knuckle_len = (hinge_w - 4.0 * knuckle_gap) / 5.0
    knuckle_radius = 0.018

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_geometry(_open_tub_mesh(width, depth, cabinet_h, wall, bottom), "hollow_cabinet_shell"),
        origin=Origin(),
        material=white,
        name="cabinet_shell",
    )
    # Slightly darker liner floor visible when the lid is raised.
    cabinet.visual(
        Box((width - 2.0 * wall - 0.010, depth - 2.0 * wall - 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, bottom + 0.003)),
        material=liner,
        name="inner_floor",
    )
    # Compressible-looking gasket on the rim.
    cabinet.visual(
        Box((width - 0.05, 0.030, 0.003)),
        origin=Origin(xyz=(0.0, front_y - 0.022, cabinet_h + 0.0015)),
        material=gasket,
        name="front_gasket",
    )
    cabinet.visual(
        Box((width - 0.05, 0.030, 0.003)),
        origin=Origin(xyz=(0.0, rear_y + 0.022, cabinet_h + 0.0015)),
        material=gasket,
        name="rear_gasket",
    )
    cabinet.visual(
        Box((0.030, depth - 0.11, 0.003)),
        origin=Origin(xyz=(-width / 2.0 + 0.022, 0.0, cabinet_h + 0.0015)),
        material=gasket,
        name="side_gasket_0",
    )
    cabinet.visual(
        Box((0.030, depth - 0.11, 0.003)),
        origin=Origin(xyz=(width / 2.0 - 0.022, 0.0, cabinet_h + 0.0015)),
        material=gasket,
        name="side_gasket_1",
    )
    # Toe kick and front service grille.
    cabinet.visual(
        Box((width - 0.08, 0.010, 0.070)),
        origin=Origin(xyz=(0.0, front_y + 0.005, 0.050)),
        material=black,
        name="toe_kick",
    )
    cabinet.visual(
        Box((0.34, 0.012, 0.055)),
        origin=Origin(xyz=(-0.30, front_y + 0.006, 0.125)),
        material=black,
        name="front_grille",
    )
    cabinet.visual(
        Box((0.055, 0.014, 0.055)),
        origin=Origin(xyz=(0.38, front_y + 0.007, 0.155)),
        material=black,
        name="drain_plug",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_depth, lid_t)),
        origin=Origin(
            xyz=(
                0.0,
                (lid_rear_y + lid_front_y) / 2.0 - hinge_y,
                (lid_bottom + lid_top) / 2.0 - hinge_z,
            )
        ),
        material=white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.40, 0.018, 0.026)),
        origin=Origin(
            xyz=(
                0.0,
                lid_front_y + 0.009 - hinge_y,
                lid_bottom + 0.022 - hinge_z,
            )
        ),
        material=liner,
        name="front_handle",
    )
    lid.visual(
        Box((lid_w - 0.16, 0.018, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                lid_front_y - 0.020 - hinge_y,
                lid_bottom + 0.004 - hinge_z,
            )
        ),
        material=gasket,
        name="lid_front_seal",
    )

    cyl_to_x = Origin(rpy=(0.0, pi / 2.0, 0.0))
    parent_segment_indices = (0, 2, 4)
    child_segment_indices = (1, 3)
    for hinge_i, hx in enumerate(hinge_xs):
        # Cabinet-side rear hinge leaf.
        cabinet.visual(
            Box((hinge_w + 0.030, 0.008, 0.085)),
            origin=Origin(xyz=(hx, rear_y - 0.004, cabinet_h - 0.020)),
            material=metal,
            name=f"hinge_leaf_{hinge_i}",
        )
        for idx in parent_segment_indices:
            x = hx - hinge_w / 2.0 + idx * (knuckle_len + knuckle_gap) + knuckle_len / 2.0
            cabinet.visual(
                Cylinder(radius=knuckle_radius, length=knuckle_len),
                origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=cyl_to_x.rpy),
                material=metal,
                name=f"fixed_knuckle_{hinge_i}_{idx}",
            )
            cabinet.visual(
                Box((knuckle_len, abs(hinge_y - rear_y) + 0.010, 0.012)),
                origin=Origin(xyz=(x, (hinge_y + rear_y) / 2.0, hinge_z - 0.018)),
                material=metal,
                name=f"fixed_strap_{hinge_i}_{idx}",
            )

        for idx in child_segment_indices:
            x = hx - hinge_w / 2.0 + idx * (knuckle_len + knuckle_gap) + knuckle_len / 2.0
            lid.visual(
                Cylinder(radius=knuckle_radius, length=knuckle_len),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=cyl_to_x.rpy),
                material=metal,
                name=f"moving_knuckle_{hinge_i}_{idx}",
            )
            lid.visual(
                Box((knuckle_len, abs(hinge_y - lid_rear_y) + 0.010, 0.010)),
                origin=Origin(xyz=(x, (lid_rear_y - hinge_y) / 2.0, -0.006)),
                material=metal,
                name=f"moving_strap_{hinge_i}_{idx}",
            )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("rear_lid_hinge")

    ctx.check(
        "single revolute lid joint",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint_type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis runs across rear edge",
        tuple(round(v, 3) for v in hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="front_gasket",
            min_gap=0.0,
            max_gap=0.008,
            name="closed lid rests just above gasket",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="xy",
            elem_a="lid_shell",
            elem_b="cabinet_shell",
            min_overlap=0.62,
            name="closed lid covers cabinet footprint",
        )
        closed_handle = ctx.part_element_world_aabb(lid, elem="front_handle")

    with ctx.pose({hinge: 1.35}):
        opened_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="front_handle",
            negative_elem="front_gasket",
            min_gap=0.45,
            name="front of lid lifts upward when opened",
        )

    ctx.check(
        "handle rises on hinge motion",
        closed_handle is not None
        and opened_handle is not None
        and opened_handle[0][2] > closed_handle[0][2] + 0.45,
        details=f"closed={closed_handle}, opened={opened_handle}",
    )

    return ctx.report()


object_model = build_object_model()
